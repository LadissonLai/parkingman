#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Keyframe-based static obstacle occupancy grid map for CARLA closed-loop tests.

Adapted from perception/scripts/lidar2grid_static.py.
Key differences:
  - world_frame defaults to 'camera_init' (defined by carla_gt_odom.py)
  - Publishes /local_map (relayed to /map by setup_closetest_main.sh for Hybrid A*)
  - Map origin defaults to (-75, -75) so the 150x150 m grid is centred on
    the camera_init origin (vehicle start position)

Depends on:
  carla_gt_odom.py  — must be running so that the map->camera_init TF exists
"""

import math
import time

import rospy
import numpy as np
import pcl
import tf2_ros
import tf.transformations as tft
import sensor_msgs.point_cloud2 as pc2

from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import (
    Pose, Point, Quaternion, PoseStamped, TransformStamped
)


class CarlaStaticMap:

    def __init__(self):
        rospy.init_node('carla_static_map', anonymous=True)

        self._load_params()

        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._map_w = int(self._map_width_m  / self._resolution)
        self._map_h = int(self._map_height_m / self._resolution)
        self._log_odds = np.zeros((self._map_h, self._map_w), dtype=np.float32)

        self._last_kf_pose = None
        self._last_kf_time = rospy.Time(0)

        self._map_pub = rospy.Publisher(
            '/local_map', OccupancyGrid, queue_size=1, latch=True
        )
        self._pts_pub = rospy.Publisher(
            '/carla_static_map/obstacle_points', PointCloud2, queue_size=1
        )

        rospy.Subscriber(
            '/carla/ego_vehicle/lidar', PointCloud2,
            self._lidar_cb, queue_size=1
        )
        rospy.loginfo("carla_static_map: waiting for LiDAR data ...")

    # ── parameters ────────────────────────────────────────────────────────────

    def _load_params(self):
        self._world_frame      = rospy.get_param('~world_frame',      'camera_init')
        self._robot_base_frame = rospy.get_param('~robot_base_frame', 'ego_vehicle')

        self._lidar_max_range   = rospy.get_param('~lidar_max_range',  50.0)
        self._range_factor      = rospy.get_param('~lidar_max_range_threshold_factor', 0.99)
        self._ransac_thresh     = rospy.get_param('~ransac_distance_threshold', 0.15)
        self._obs_min_z         = rospy.get_param('~obstacle_min_height_world', 0.3)
        self._obs_max_z         = rospy.get_param('~obstacle_max_height_world', 1.0)

        self._resolution        = rospy.get_param('~resolution',  0.1)
        self._map_width_m       = rospy.get_param('~width',       150.0)
        self._map_height_m      = rospy.get_param('~height',      150.0)
        # Centre map on camera_init origin
        self._origin_x          = rospy.get_param('~origin_x',   -75.0)
        self._origin_y          = rospy.get_param('~origin_y',   -75.0)

        self._lo_occupied       = rospy.get_param('~log_odds_occupied',  0.9)
        self._lo_clamp_min      = rospy.get_param('~log_odds_clamp_min', -5.0)
        self._lo_clamp_max      = rospy.get_param('~log_odds_clamp_max',  5.0)

        self._kf_dist_thresh    = rospy.get_param('~keyframe_dist_thresh',  0.5)
        self._kf_angle_thresh   = rospy.get_param('~keyframe_angle_thresh', 10.0)
        self._kf_time_thresh    = rospy.get_param('~keyframe_time_thresh',   2.0)

    # ── LiDAR callback ────────────────────────────────────────────────────────

    def _lidar_cb(self, msg):
        try:
            curr_pose = self._get_current_pose(msg.header.stamp)
            if curr_pose is None:
                return
            if self._is_keyframe(curr_pose, msg.header.stamp):
                t0 = time.time()
                self._process_and_update(msg)
                rospy.loginfo("carla_static_map: keyframe processed in %.1f ms",
                              (time.time() - t0) * 1000)
                self._last_kf_pose = curr_pose
                self._last_kf_time = msg.header.stamp
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(5.0, "carla_static_map: TF lookup failed: %s", e)
        except Exception as e:
            rospy.logerr("carla_static_map: error processing keyframe: %s", e)

    # ── map update ────────────────────────────────────────────────────────────

    def _process_and_update(self, msg):
        # Read LiDAR points into numpy array
        pts = np.array(
            list(pc2.read_points(msg, skip_nans=True, field_names=('x', 'y', 'z'))),
            dtype=np.float32
        )
        if pts.shape[0] == 0:
            return

        # Range filter
        dists = np.linalg.norm(pts, axis=1)
        pts = pts[dists < self._lidar_max_range * self._range_factor]
        if pts.shape[0] == 0:
            return

        # RANSAC ground removal
        cloud = pcl.PointCloud(pts)
        seg   = cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(self._ransac_thresh)
        inliers, _ = seg.segment()
        if inliers:
            pts = np.delete(pts, inliers, axis=0)
        if pts.shape[0] == 0:
            return

        # Transform to world (camera_init) frame
        tf_stamp = self._tf_buffer.lookup_transform(
            self._world_frame, msg.header.frame_id,
            msg.header.stamp, rospy.Duration(1.0)
        )
        pts_world = self._transform_points(pts, tf_stamp)

        # Height slice
        z = pts_world[:, 2]
        pts_world = pts_world[(z > self._obs_min_z) & (z < self._obs_max_z)]
        if pts_world.shape[0] == 0:
            return

        # Publish debug point cloud
        debug_msg = pc2.create_cloud_xyz32(tf_stamp.header, pts_world)
        self._pts_pub.publish(debug_msg)

        self._update_grid(pts_world)
        self._publish_map()

    @staticmethod
    def _transform_points(pts, tf_stamped):
        """Apply a geometry_msgs/TransformStamped to an (N,3) numpy array."""
        t   = tf_stamped.transform.translation
        rot = tf_stamped.transform.rotation
        mat = tft.quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
        mat[0:3, 3] = [t.x, t.y, t.z]
        pts_h = np.hstack((pts, np.ones((pts.shape[0], 1))))
        return (mat @ pts_h.T).T[:, :3]

    def _update_grid(self, pts_world):
        mx = ((pts_world[:, 0] - self._origin_x) / self._resolution).astype(np.int32)
        my = ((pts_world[:, 1] - self._origin_y) / self._resolution).astype(np.int32)
        mask = (mx >= 0) & (mx < self._map_w) & (my >= 0) & (my < self._map_h)
        np.add.at(self._log_odds, (my[mask], mx[mask]), self._lo_occupied)
        np.clip(self._log_odds, self._lo_clamp_min, self._lo_clamp_max,
                out=self._log_odds)

    def _publish_map(self):
        prob = 1.0 - 1.0 / (1.0 + np.exp(self._log_odds))
        occ  = (prob * 100).astype(np.int8)
        occ[self._log_odds == 0] = 10   # unknown cells shown as 10%

        grid = OccupancyGrid()
        grid.header.stamp    = rospy.Time.now()
        grid.header.frame_id = self._world_frame
        grid.info.resolution = self._resolution
        grid.info.width      = self._map_w
        grid.info.height     = self._map_h
        grid.info.origin     = Pose(
            Point(self._origin_x, self._origin_y, 0.0),
            Quaternion(0, 0, 0, 1)
        )
        grid.data = occ.flatten().tolist()
        self._map_pub.publish(grid)
        rospy.loginfo_once("carla_static_map: first /local_map published")

    # ── keyframe helpers ──────────────────────────────────────────────────────

    def _get_current_pose(self, stamp):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._world_frame, self._robot_base_frame,
                stamp, rospy.Duration(1.0)
            )
            ps = PoseStamped()
            ps.header = tf.header
            ps.pose.position    = tf.transform.translation
            ps.pose.orientation = tf.transform.rotation
            return ps
        except Exception as e:
            rospy.logwarn_throttle(5.0,
                "carla_static_map: cannot get current pose: %s", e)
            return None

    def _is_keyframe(self, curr, stamp):
        if self._last_kf_pose is None:
            return True

        dx   = curr.pose.position.x - self._last_kf_pose.pose.position.x
        dy   = curr.pose.position.y - self._last_kf_pose.pose.position.y
        dist = math.hypot(dx, dy)
        if dist > self._kf_dist_thresh:
            return True

        def _yaw(pose):
            o = pose.pose.orientation
            _, _, y = tft.euler_from_quaternion([o.x, o.y, o.z, o.w])
            return math.degrees(y)

        da = abs(_yaw(curr) - _yaw(self._last_kf_pose))
        if da > 180:
            da = 360 - da
        if da > self._kf_angle_thresh:
            return True

        if (stamp - self._last_kf_time).to_sec() > self._kf_time_thresh:
            return True

        return False


def main():
    node = CarlaStaticMap()
    rospy.spin()


if __name__ == '__main__':
    main()

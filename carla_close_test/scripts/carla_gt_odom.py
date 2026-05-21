#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Replaces FastLIO odometry with CARLA ground-truth odometry.

Subscribes to /carla/ego_vehicle/odometry (map frame).
On the first message, records the initial pose as the camera_init origin,
then publishes a static TF map -> camera_init.
Each subsequent message is converted into the camera_init frame and
re-published as /Odometry_camera_init for closetest_main.py to consume.

ROS topics published:
  /Odometry_camera_init  (nav_msgs/Odometry, frame=camera_init, child=body)

TF published:
  map -> camera_init  (static, once)
"""

import math
import threading

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler


def _quat_to_yaw(qx, qy, qz, qw):
    _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
    return yaw


def _yaw_to_quat(yaw):
    return quaternion_from_euler(0.0, 0.0, yaw)


class CarlaGtOdom:

    def __init__(self):
        rospy.init_node('carla_gt_odom', anonymous=False)

        self._lock        = threading.Lock()
        self._origin_x    = None
        self._origin_y    = None
        self._origin_z    = None
        self._origin_yaw  = None

        self._static_br = tf2_ros.StaticTransformBroadcaster()

        self._odom_pub = rospy.Publisher(
            '/Odometry_camera_init', Odometry, queue_size=50
        )

        rospy.Subscriber(
            '/carla/ego_vehicle/odometry', Odometry,
            self._cb, queue_size=50
        )
        rospy.loginfo("carla_gt_odom: waiting for /carla/ego_vehicle/odometry ...")

    # ── callback ──────────────────────────────────────────────────────────────

    def _cb(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        z   = msg.pose.pose.position.z
        q   = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

        with self._lock:
            if self._origin_x is None:
                self._origin_x   = x
                self._origin_y   = y
                self._origin_z   = z
                self._origin_yaw = yaw
                self._publish_static_tf(x, y, z, yaw)
                rospy.loginfo(
                    "carla_gt_odom: camera_init origin = map(%.3f, %.3f, yaw=%.3f rad)",
                    x, y, yaw
                )

            ox, oy, oyaw = self._origin_x, self._origin_y, self._origin_yaw

        # Pose in camera_init frame: T_ci = R0^T * (p - p0)
        dx = x - ox
        dy = y - oy
        ci_x   =  dx * math.cos(oyaw) + dy * math.sin(oyaw)
        ci_y   = -dx * math.sin(oyaw) + dy * math.cos(oyaw)
        ci_yaw =  yaw - oyaw

        out = Odometry()
        out.header.stamp    = msg.header.stamp
        out.header.frame_id = 'camera_init'
        out.child_frame_id  = 'body'

        out.pose.pose.position.x = ci_x
        out.pose.pose.position.y = ci_y
        out.pose.pose.position.z = 0.0

        q_out = _yaw_to_quat(ci_yaw)
        out.pose.pose.orientation.x = q_out[0]
        out.pose.pose.orientation.y = q_out[1]
        out.pose.pose.orientation.z = q_out[2]
        out.pose.pose.orientation.w = q_out[3]

        out.twist = msg.twist

        self._odom_pub.publish(out)

    # ── TF ────────────────────────────────────────────────────────────────────

    def _publish_static_tf(self, ox, oy, oz, oyaw):
        """camera_init origin = initial vehicle pose in map frame."""
        ts = TransformStamped()
        ts.header.stamp    = rospy.Time.now()
        ts.header.frame_id = 'map'
        ts.child_frame_id  = 'camera_init'

        ts.transform.translation.x = ox
        ts.transform.translation.y = oy
        ts.transform.translation.z = oz

        q = _yaw_to_quat(oyaw)
        ts.transform.rotation.x = q[0]
        ts.transform.rotation.y = q[1]
        ts.transform.rotation.z = q[2]
        ts.transform.rotation.w = q[3]

        self._static_br.sendTransform(ts)
        rospy.loginfo("carla_gt_odom: static TF map -> camera_init published")


def main():
    node = CarlaGtOdom()
    rospy.spin()


if __name__ == '__main__':
    main()

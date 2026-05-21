#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Closed-loop VLM parking navigation test node.

Workflow:
  1. Read task folder (gt_trajectory.csv, instruct.txt)
  2. Maintain keyframe history from /Odometry_camera_init
  3. Accumulate global parking spaces from /parking_map/confirmed_spaces
  4. Loop: call VLM -> explore (Pure Pursuit + Ackermann) or park (Hybrid A* + set_transform)
  5. Evaluate performance and save to performance_result/

Usage:
  rosrun carla_close_test closetest_main.py \
    _task_dir:=/path/to/task _vlm_server:=http://localhost:9999

ROS Params (all optional, have defaults):
  ~task_dir              path to task folder
  ~vlm_server            VLM FastAPI server URL
  ~set_transform_rate    Hz for set_transform in park_navigate (default 2.0)
  ~arc_interp_step       arc spatial resolution m/point (default 0.05)
  ~arc_execute_rate      Hz for set_transform in arc_navigate (default 50.0)
  ~arc_waypoint_index    0-based index of VLM waypoint used as arc target (default 3)
  ~max_vlm_steps         max VLM loop iterations (default 50)
  ~kf_dist_thresh        keyframe distance threshold m (default 0.1)
  ~kf_yaw_thresh         keyframe yaw threshold deg (default 5.0)
  ~correct_space_thresh  APE threshold for correct space m (default 3.0)
  ~hybrid_astar_timeout  seconds to wait for A* path (default 30.0)
  ~task_timeout          max seconds for whole task (default 600.0)
"""

import os
import sys
import math
import time
import base64
import threading
import datetime

import rospy
import numpy as np
import pandas as pd
import requests
from io import BytesIO

from PIL import Image as PILImage

from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import Image as RosImage
from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from parking_space_msgs.msg import ParkingSpaceArray

# Collision sensor (carla_msgs)
try:
    from carla_msgs.msg import CarlaCollisionEvent
    _HAS_CARLA_MSGS = True
except ImportError:
    _HAS_CARLA_MSGS = False
    rospy.logwarn("carla_msgs not found — collision detection disabled")


# ─────────────────────────────────────────────────────────────────────────────
# Helpers
# ─────────────────────────────────────────────────────────────────────────────

def _ros_image_to_b64(ros_img):
    """Convert sensor_msgs/Image (BGR8 or RGB8) to base64-encoded PNG string."""
    h, w = ros_img.height, ros_img.width
    encoding = ros_img.encoding.lower()
    data = np.frombuffer(ros_img.data, dtype=np.uint8).reshape(h, w, -1)
    if 'bgr' in encoding:
        data = data[:, :, ::-1]  # BGR -> RGB
    pil = PILImage.fromarray(data[:, :, :3], 'RGB')
    buf = BytesIO()
    pil.save(buf, format='PNG')
    return base64.b64encode(buf.getvalue()).decode('utf-8')


def _quat_to_yaw(qx, qy, qz, qw):
    """Extract yaw (radians) from quaternion."""
    _, _, yaw = euler_from_quaternion([qx, qy, qz, qw])
    return yaw


def _yaw_to_quat(yaw):
    """Convert yaw (radians) to quaternion [x,y,z,w]."""
    return quaternion_from_euler(0.0, 0.0, yaw)


def _path_length(poses_xy):
    """Sum of Euclidean distances for a list of (x, y) tuples."""
    total = 0.0
    for i in range(1, len(poses_xy)):
        dx = poses_xy[i][0] - poses_xy[i-1][0]
        dy = poses_xy[i][1] - poses_xy[i-1][1]
        total += math.hypot(dx, dy)
    return total


def _normalize_angle_deg(a):
    """Normalise angle to (-180, 180]."""
    a = a % 360.0
    if a > 180.0:
        a -= 360.0
    return a


# ─────────────────────────────────────────────────────────────────────────────
# KeyframeTracker
# ─────────────────────────────────────────────────────────────────────────────

class KeyframeTracker:
    """
    Subscribes to /Odometry_camera_init at 2 Hz and records keyframes.
    A new keyframe is saved when distance > kf_dist_thresh OR
    yaw change > kf_yaw_thresh from the last recorded keyframe.
    Poses are stored in camera_init frame as {x, y, yaw_deg}.
    """

    def __init__(self, kf_dist_thresh=0.1, kf_yaw_thresh=5.0):
        self._lock = threading.Lock()
        self._history = []        # list of {x, y, yaw_deg} — excludes current frame
        self._last_kf_x = None
        self._last_kf_y = None
        self._last_kf_yaw = None  # radians
        self._curr_odom = None    # latest Odometry msg

        self._dist_thresh = kf_dist_thresh
        self._yaw_thresh  = math.radians(kf_yaw_thresh)

        rospy.Subscriber('/Odometry_camera_init', Odometry,
                         self._odom_cb, queue_size=20)
        rospy.Timer(rospy.Duration(0.5), self._tick)

    def _odom_cb(self, msg):
        with self._lock:
            self._curr_odom = msg

    def _tick(self, _event):
        with self._lock:
            if self._curr_odom is None:
                return
            msg = self._curr_odom
        self._try_add_keyframe(msg)

    def _try_add_keyframe(self, msg):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)

        with self._lock:
            if self._last_kf_x is None:
                # First keyframe — always add
                self._save_keyframe(x, y, yaw)
                return

            dist = math.hypot(x - self._last_kf_x, y - self._last_kf_y)
            yaw_diff = abs(yaw - self._last_kf_yaw)
            # Wrap to [0, pi]
            if yaw_diff > math.pi:
                yaw_diff = 2 * math.pi - yaw_diff

            if dist > self._dist_thresh or yaw_diff > self._yaw_thresh:
                self._save_keyframe(x, y, yaw)

    def _save_keyframe(self, x, y, yaw):
        """Must be called with self._lock held."""
        self._history.append({
            'x':       x,
            'y':       y,
            'yaw':     math.degrees(yaw),  # VLM server expects degrees
        })
        self._last_kf_x   = x
        self._last_kf_y   = y
        self._last_kf_yaw = yaw

    def get_history(self):
        """Return all keyframes excluding the most recent (which is curr_pose)."""
        with self._lock:
            if len(self._history) <= 1:
                return []
            return list(self._history[:-1])

    def get_curr_odom(self):
        with self._lock:
            return self._curr_odom

    def get_trajectory_xy(self):
        """Return list of (x, y) for all keyframes — used for SPL computation."""
        with self._lock:
            return [(kf['x'], kf['y']) for kf in self._history]

    def ready(self):
        with self._lock:
            return self._curr_odom is not None


# ─────────────────────────────────────────────────────────────────────────────
# ParkingSpaceManager
# ─────────────────────────────────────────────────────────────────────────────

class ParkingSpaceManager:
    """
    Subscribes to /parking_map/confirmed_spaces (camera_init frame).
    Accumulates all seen spaces into a dict keyed by id.
    """

    def __init__(self):
        self._lock   = threading.Lock()
        self._spaces = {}  # id -> ParkingSpace msg

        rospy.Subscriber('/parking_map/confirmed_spaces',
                         ParkingSpaceArray, self._cb, queue_size=10)

    def _cb(self, msg):
        with self._lock:
            for sp in msg.spaces:
                self._spaces[sp.id] = sp

    def get_all_slots(self):
        """Return list of {id, x, y} in camera_init frame."""
        with self._lock:
            return [
                {'id': sid, 'x': sp.pose.position.x, 'y': sp.pose.position.y}
                for sid, sp in self._spaces.items()
            ]

    def get_all_slots_body(self, curr_odom):
        """Return list of {id, x, y} transformed to ego body frame for VLM input.

        camera_init -> body:
            dx, dy = p_ci - origin_ci
            bx =  dx*cos(yaw) + dy*sin(yaw)
            by = -dx*sin(yaw) + dy*cos(yaw)
        """
        ox   = curr_odom.pose.pose.position.x
        oy   = curr_odom.pose.pose.position.y
        q    = curr_odom.pose.pose.orientation
        oyaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        cos_yaw = math.cos(oyaw)
        sin_yaw = math.sin(oyaw)
        with self._lock:
            result = []
            for sid, sp in self._spaces.items():
                dx = sp.pose.position.x - ox
                dy = sp.pose.position.y - oy
                result.append({
                    'id': sid,
                    'x':  dx * cos_yaw + dy * sin_yaw,
                    'y': -dx * sin_yaw + dy * cos_yaw,
                })
            return result

    def get_space_by_id(self, sid):
        """Return full ParkingSpace msg or None."""
        with self._lock:
            return self._spaces.get(int(sid), None)

    def ready(self):
        with self._lock:
            return len(self._spaces) > 0


# ─────────────────────────────────────────────────────────────────────────────
# ImageBuffer
# ─────────────────────────────────────────────────────────────────────────────

class ImageBuffer:
    """
    Maintains latest base64 PNG for front/rear/left/right cameras.
    No resize — server's PrismaticImageProcessor handles that internally.
    """

    TOPICS = {
        'front': '/carla/ego_vehicle/rgb_front/image',
        'rear':  '/carla/ego_vehicle/rgb_rear/image',
        'left':  '/carla/ego_vehicle/rgb_left/image',
        'right': '/carla/ego_vehicle/rgb_right/image',
    }

    def __init__(self):
        self._lock   = threading.Lock()
        self._images = {k: None for k in self.TOPICS}

        for view, topic in self.TOPICS.items():
            rospy.Subscriber(topic, RosImage,
                             lambda msg, v=view: self._cb(msg, v),
                             queue_size=2)

    def _cb(self, msg, view):
        b64 = _ros_image_to_b64(msg)
        with self._lock:
            self._images[view] = b64

    def get_all(self):
        """Return dict of {front, rear, left, right} -> b64 string, or None if not ready."""
        with self._lock:
            if any(v is None for v in self._images.values()):
                return None
            return dict(self._images)

    def ready(self):
        with self._lock:
            return all(v is not None for v in self._images.values())


# ─────────────────────────────────────────────────────────────────────────────
# VLMClient
# ─────────────────────────────────────────────────────────────────────────────

class VLMClient:
    """POST requests to FastAPI VLM server at /predict."""

    def __init__(self, server_url, timeout=120):
        self._url     = server_url.rstrip('/') + '/predict'
        self._timeout = timeout

    def predict(self, instruction, history_poses, curr_pose, parking_slots, images):
        """
        Args:
            instruction    : str
            history_poses  : list of {x, y, yaw}  (camera_init, yaw in degrees)
            curr_pose      : {x, y, yaw}           (camera_init, yaw in degrees)
            parking_slots  : list of {id, x, y}
            images         : {front, rear, left, right} -> b64 PNG strings
        Returns:
            (decision_id: int, trajectory: list of {x,y,cos_yaw,sin_yaw})
        """
        payload = {
            'instruction':      instruction,
            'image_front_b64':  images['front'],
            'image_rear_b64':   images['rear'],
            'image_left_b64':   images['left'],
            'image_right_b64':  images['right'],
            'history_poses':    history_poses,
            'curr_pose':        curr_pose,
            'parking_slots':    parking_slots,
        }
        resp = requests.post(self._url, json=payload, timeout=self._timeout)
        resp.raise_for_status()
        data = resp.json()
        return int(data['decision_id']), data['trajectory']


# ─────────────────────────────────────────────────────────────────────────────
# NavigationExecutor
# ─────────────────────────────────────────────────────────────────────────────

class NavigationExecutor:
    """
    Publishes geometry_msgs/Pose to /carla/ego_vehicle/control/set_transform.
    Handles body->camera_init->map conversion and interpolation.
    """

    def __init__(self, set_transform_rate=2.0,
                 arc_interp_step=0.05, arc_execute_rate=50.0,
                 arc_waypoint_index=3):
        self._sleep_dur          = 1.0 / set_transform_rate   # park_navigate rate
        self._arc_interp_step    = arc_interp_step
        self._arc_execute_rate   = arc_execute_rate
        self._arc_waypoint_index = arc_waypoint_index

        self._pub = rospy.Publisher(
            '/carla/ego_vehicle/control/set_transform',
            Pose, queue_size=10
        )
        self._traj_marker_pub = rospy.Publisher(
            '/vlm_trajectory_marker', Marker, queue_size=10
        )
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf)

    def _publish_pose_camera_init(self, ci_x, ci_y, ci_yaw):
        """Transform one camera_init pose to map and publish (no sleep — callers control rate)."""
        ps = PoseStamped()
        ps.header.stamp    = rospy.Time.now()
        ps.header.frame_id = 'camera_init'
        ps.pose.position.x = ci_x
        ps.pose.position.y = ci_y
        ps.pose.position.z = 0.0
        q = _yaw_to_quat(ci_yaw)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]

        try:
            tf_ps = self._tf_buf.transform(ps, 'map', rospy.Duration(1.0))
        except Exception as e:
            rospy.logwarn("TF camera_init->map failed: %s", e)
            return

        pose = Pose()
        pose.position    = tf_ps.pose.position
        pose.orientation = tf_ps.pose.orientation
        if pose.position.z < 0.25:
            pose.position.z = 0.25
        self._pub.publish(pose)

    def _publish_traj_marker(self, ci_pts):
        """Publish VLM trajectory as a red LINE_STRIP in camera_init frame.
        Fixed id=0 so each new trajectory replaces the previous one in RViz."""
        if not ci_pts:
            return
        m = Marker()
        m.header.stamp    = rospy.Time.now()
        m.header.frame_id = 'camera_init'
        m.ns              = 'vlm_trajectory'
        m.id              = 0             # fixed id: replaces previous marker
        m.type            = Marker.LINE_STRIP
        m.action          = Marker.ADD
        m.scale.x         = 0.15
        m.color.r         = 1.0
        m.color.g         = 0.0
        m.color.b         = 0.0
        m.color.a         = 1.0
        m.lifetime        = rospy.Duration(0)
        for (x, y, _) in ci_pts:
            p = Point()
            p.x = x
            p.y = y
            p.z = 0.3
            m.points.append(p)
        self._traj_marker_pub.publish(m)

    def arc_navigate(self, trajectory_body, curr_odom, had_collision_fn=None):
        """
        Navigate to the 4th VLM waypoint via a circular arc using set_transform.

        Converts the body-frame waypoint to camera_init, computes a circular arc
        from current pose to target (yaw at arrival matches VLM waypoint yaw),
        then executes each interpolated point via set_transform at arc_execute_rate Hz.

        Args:
            trajectory_body  : list of {x, y, cos_yaw, sin_yaw} in body frame
            curr_odom        : Odometry snapshot at VLM call time
            had_collision_fn : Callable -> bool, checked each waypoint

        Returns:
            True if a collision was detected, False otherwise.
        """
        if not trajectory_body:
            rospy.logwarn("Empty VLM trajectory — skipping arc step")
            return False

        wp   = trajectory_body[min(self._arc_waypoint_index, len(trajectory_body) - 1)]
        ox   = curr_odom.pose.pose.position.x
        oy   = curr_odom.pose.pose.position.y
        oyaw = _quat_to_yaw(*[getattr(curr_odom.pose.pose.orientation, a)
                               for a in ('x', 'y', 'z', 'w')])

        # Convert 4th waypoint: body -> camera_init (position + yaw)
        bx, by    = wp['x'], wp['y']
        body_yaw  = math.atan2(wp['sin_yaw'], wp['cos_yaw'])
        tx   = ox + bx * math.cos(oyaw) - by * math.sin(oyaw)
        ty   = oy + bx * math.sin(oyaw) + by * math.cos(oyaw)
        tyaw = oyaw + body_yaw

        rospy.loginfo("Arc target (camera_init): (%.2f, %.2f, %.1f°)",
                      tx, ty, math.degrees(tyaw))

        pts = self._interpolate_arc(ox, oy, oyaw, tx, ty, tyaw)

        # Visualise arc as red trajectory marker
        self._publish_traj_marker([(p[0], p[1], 0.0) for p in pts])

        arc_sleep = 1.0 / self._arc_execute_rate
        for (xi, yi, ti) in pts:
            if rospy.is_shutdown():
                break
            if had_collision_fn is not None and had_collision_fn():
                rospy.logwarn("Collision detected during arc navigation")
                return True
            self._publish_pose_camera_init(xi, yi, ti)
            time.sleep(arc_sleep)

        return False

    def _interpolate_arc(self, x0, y0, t0, x1, y1, t1):
        """
        Circular arc from (x0,y0,t0) to (x1,y1,t1) with yaw matching at arrival.
        Falls back to linear interpolation for near-straight or near-zero paths.
        Returns list of (x, y, yaw) tuples in camera_init frame.
        """
        chord = math.hypot(x1 - x0, y1 - y0)
        if chord < 1e-2:
            return [(x0, y0, t0), (x1, y1, t1)]

        # Normalise delta yaw to (-pi, pi]
        dt = t1 - t0
        dt = (dt + math.pi) % (2.0 * math.pi) - math.pi

        if abs(dt) < 1e-3:
            # Straight line
            n = max(2, int(math.ceil(chord / self._arc_interp_step)))
            return [
                (x0 + (x1 - x0) * k / n,
                 y0 + (y1 - y0) * k / n,
                 t0 + dt * k / n)
                for k in range(n + 1)
            ]

        # Circular arc
        R    = chord / (2.0 * abs(math.sin(dt / 2.0)))
        sign = 1.0 if dt > 0 else -1.0
        cx   = x0 - sign * R * math.sin(t0)
        cy   = y0 + sign * R * math.cos(t0)

        arc_len = R * abs(dt)
        n = max(2, int(math.ceil(arc_len / self._arc_interp_step)))

        alpha0 = math.atan2(y0 - cy, x0 - cx)
        pts = []
        for k in range(n + 1):
            frac   = k / n
            alpha  = alpha0 + frac * dt
            pts.append((cx + R * math.cos(alpha),
                        cy + R * math.sin(alpha),
                        t0 + frac * dt))
        return pts

    def park_navigate(self, path_msg, had_collision_fn=None):
        """
        Follow Hybrid A* path (nav_msgs/Path, frame_id=camera_init) via set_transform.
        Silences Ackermann control first (approach B) so the two actuators don't conflict.

        Args:
            path_msg         : nav_msgs/Path in camera_init frame
            had_collision_fn : Callable -> bool, checked before each waypoint

        Returns:
            True if a collision was detected during parking, False otherwise.
        """
        rospy.loginfo("Following Hybrid A* path with %d points", len(path_msg.poses))
        for ps in path_msg.poses:
            if rospy.is_shutdown():
                break
            if had_collision_fn is not None and had_collision_fn():
                rospy.logwarn("Collision detected during parking — aborting path execution")
                return True
            ci_x   = ps.pose.position.x
            ci_y   = ps.pose.position.y
            q      = ps.pose.orientation
            ci_yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
            self._publish_pose_camera_init(ci_x, ci_y, ci_yaw)
            time.sleep(self._sleep_dur)

        return False


# ─────────────────────────────────────────────────────────────────────────────
# HybridAStarInterface
# ─────────────────────────────────────────────────────────────────────────────

class HybridAStarInterface:
    """
    Publishes start/goal to Hybrid A* and waits for path result.
    All poses in camera_init frame.
    """

    def __init__(self, timeout=30.0):
        self._timeout  = timeout
        self._path     = None
        self._lock     = threading.Lock()

        self._start_pub = rospy.Publisher(
            '/parkman/planning/input/start',
            PoseStamped, queue_size=1
        )
        self._goal_pub = rospy.Publisher(
            '/parkman/planning/input/goal',
            PoseStamped, queue_size=1
        )
        rospy.Subscriber('/parkman/planning/output/trajectory',
                         Path, self._path_cb, queue_size=1)

    def _path_cb(self, msg):
        with self._lock:
            self._path = msg

    def _make_pose_stamped(self, x, y, yaw_rad):
        ps = PoseStamped()
        ps.header.stamp    = rospy.Time.now()
        ps.header.frame_id = 'camera_init'
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = 0.0
        q = _yaw_to_quat(yaw_rad)
        ps.pose.orientation.x = q[0]
        ps.pose.orientation.y = q[1]
        ps.pose.orientation.z = q[2]
        ps.pose.orientation.w = q[3]
        return ps

    def request_path(self, curr_odom, goal_space):
        """
        Request a path from current pose to goal_space (ParkingSpace msg).
        Returns nav_msgs/Path or raises RuntimeError on timeout.
        """
        # Reset previous path
        with self._lock:
            self._path = None

        # Start pose from current odometry
        sx   = curr_odom.pose.pose.position.x
        sy   = curr_odom.pose.pose.position.y
        sq   = curr_odom.pose.pose.orientation
        syaw = _quat_to_yaw(sq.x, sq.y, sq.z, sq.w)

        # Goal pose from ParkingSpace.pose (already in camera_init)
        gx   = goal_space.pose.position.x
        gy   = goal_space.pose.position.y
        gq   = goal_space.pose.orientation
        gyaw = _quat_to_yaw(gq.x, gq.y, gq.z, gq.w)

        rospy.loginfo(
            "Requesting Hybrid A*: start=(%.2f,%.2f,%.1f°) goal=(%.2f,%.2f,%.1f°)",
            sx, sy, math.degrees(syaw), gx, gy, math.degrees(gyaw)
        )

        start_ps = self._make_pose_stamped(sx, sy, syaw)
        goal_ps  = self._make_pose_stamped(gx, gy, gyaw)

        # Publish multiple times to ensure receipt
        for _ in range(3):
            self._start_pub.publish(start_ps)
            self._goal_pub.publish(goal_ps)
            time.sleep(0.1)

        # Wait for path
        deadline = time.time() + self._timeout
        while time.time() < deadline:
            with self._lock:
                if self._path is not None:
                    rospy.loginfo("Hybrid A* path received: %d points", len(self._path.poses))
                    return self._path
            time.sleep(0.2)

        raise RuntimeError("Hybrid A* timed out after %.1fs" % self._timeout)


# ─────────────────────────────────────────────────────────────────────────────
# CollisionMonitor
# ─────────────────────────────────────────────────────────────────────────────

class CollisionMonitor:

    def __init__(self):
        self._collision = False
        if _HAS_CARLA_MSGS:
            rospy.Subscriber('/carla/ego_vehicle/collision',
                             CarlaCollisionEvent, self._cb, queue_size=10)
        else:
            rospy.logwarn("CollisionMonitor: carla_msgs unavailable, collision detection off")

    def _cb(self, _msg):
        if not self._collision:
            rospy.logwarn("COLLISION DETECTED")
        self._collision = True

    def had_collision(self):
        return self._collision


# ─────────────────────────────────────────────────────────────────────────────
# PerformanceEvaluator
# ─────────────────────────────────────────────────────────────────────────────

_PKG_DIR = os.path.join(os.path.dirname(__file__), '..')

class PerformanceEvaluator:

    def __init__(self, correct_space_thresh=3.0):
        self._thresh = correct_space_thresh

    def compute_and_save(self, task_dir, gt_df, actual_xy, final_pose_map,
                         had_collision, parking_attempted, timeout=False):
        """
        Args:
            task_dir        : str, path to task folder
            gt_df           : DataFrame with columns x,y,z,yaw (map frame)
            actual_xy       : list of (x,y) keyframe positions (camera_init — metric)
            final_pose_map  : (x, y, yaw_rad) in map frame after parking, or None
            had_collision   : bool
            parking_attempted: bool (True if VLM triggered park decision)
            timeout         : bool
        """
        task_name = os.path.basename(os.path.normpath(task_dir))
        result_dir = os.path.join(_PKG_DIR, 'performance_result', task_name)
        os.makedirs(result_dir, exist_ok=True)

        gt_xy  = list(zip(gt_df['x'].values, gt_df['y'].values))
        gt_len = _path_length(gt_xy)
        act_len = _path_length(actual_xy) if len(actual_xy) >= 2 else 0.0

        gt_final_x   = float(gt_df.iloc[-1]['x'])
        gt_final_y   = float(gt_df.iloc[-1]['y'])
        gt_final_yaw = float(gt_df.iloc[-1]['yaw'])  # radians

        # Correct space & APE
        if final_pose_map is not None:
            fx, fy, fyaw = final_pose_map
            ape = math.hypot(fx - gt_final_x, fy - gt_final_y)
            correct_space = ape < self._thresh

            # AOE: handle head-in / head-out ambiguity
            yaw_diff = abs(math.degrees(fyaw) - math.degrees(gt_final_yaw))
            yaw_diff = yaw_diff % 360.0
            if yaw_diff > 180.0:
                yaw_diff = 360.0 - yaw_diff
            aoe = min(yaw_diff, abs(180.0 - yaw_diff))
        else:
            ape           = float('nan')
            aoe           = float('nan')
            correct_space = False

        success = (not had_collision) and correct_space and parking_attempted

        # SPL
        if success and gt_len > 0:
            spl = gt_len / max(act_len, gt_len)
        else:
            spl = 0.0

        ts  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        out = os.path.join(result_dir, 'closetest_result_%s.txt' % ts)

        lines = [
            '=== Closed-Loop Test Result ===',
            'Task:              %s' % task_name,
            'Timestamp:         %s' % ts,
            '',
            '--- Conditions ---',
            'Timeout:           %s' % timeout,
            'Collision:         %s' % had_collision,
            'Parking attempted: %s' % parking_attempted,
            '',
            '--- Metrics ---',
            'Correct space:     %s  (APE threshold: %.2f m)' % (correct_space, self._thresh),
            'Success (SR):      %s' % success,
            'SPL:               %.4f' % spl,
            'APE:               %.4f m' % ape,
            'AOE:               %.4f deg' % aoe,
            '',
            '--- Details ---',
            'GT trajectory len: %.4f m' % gt_len,
            'Actual traj len:   %.4f m' % act_len,
            'GT final pos:      (%.4f, %.4f) yaw=%.4f rad' % (gt_final_x, gt_final_y, gt_final_yaw),
            'Final pos:         %s' % (
                '(%.4f, %.4f) yaw=%.4f rad' % (fx, fy, fyaw) if final_pose_map else 'N/A'
            ),
        ]
        with open(out, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        rospy.loginfo("Performance saved to: %s", out)
        for line in lines:
            rospy.loginfo(line)

        return {
            'task':           task_name,
            'correct_space':  correct_space,
            'success':        success,
            'collision':      had_collision,
            'spl':            spl,
            'ape':            ape,
            'aoe':            aoe,
        }


# ─────────────────────────────────────────────────────────────────────────────
# ClosedLoopTestNode
# ─────────────────────────────────────────────────────────────────────────────

class ClosedLoopTestNode:

    def __init__(self):
        rospy.init_node('closetest_main', anonymous=False)

        # ── Parameters ────────────────────────────────────────────────────────
        self.task_dir     = rospy.get_param('~task_dir', '')
        vlm_server        = rospy.get_param('~vlm_server',           'http://localhost:9999')
        st_rate           = rospy.get_param('~set_transform_rate',    2.0)
        arc_interp_step   = rospy.get_param('~arc_interp_step',       0.05)
        arc_execute_rate  = rospy.get_param('~arc_execute_rate',      10.0)
        arc_wp_index      = rospy.get_param('~arc_waypoint_index',    3)            # 第几个点进行轨迹追踪
        self.max_steps    = rospy.get_param('~max_vlm_steps',        50)
        kf_dist           = rospy.get_param('~kf_dist_thresh',       0.1)
        kf_yaw            = rospy.get_param('~kf_yaw_thresh',        5.0)
        cs_thresh         = rospy.get_param('~correct_space_thresh', 3.0)
        ha_timeout        = rospy.get_param('~hybrid_astar_timeout', 30.0)
        self.task_timeout = rospy.get_param('~task_timeout',         600.0)

        if not self.task_dir:
            rospy.logerr("~task_dir not set. Exiting.")
            sys.exit(1)

        # ── Load task data ─────────────────────────────────────────────────────
        gt_csv = os.path.join(self.task_dir, 'gt_trajectory.csv')
        inst_f = os.path.join(self.task_dir, 'instruct.txt')

        if not os.path.isfile(gt_csv):
            rospy.logerr("gt_trajectory.csv not found: %s", gt_csv)
            sys.exit(1)
        if not os.path.isfile(inst_f):
            rospy.logerr("instruct.txt not found: %s", inst_f)
            sys.exit(1)

        self.gt_df = pd.read_csv(gt_csv)
        with open(inst_f, 'r', encoding='utf-8') as f:
            self.instruction = f.read().strip()

        rospy.loginfo("Task dir:    %s", self.task_dir)
        rospy.loginfo("Instruction: %s", self.instruction)

        # ── Modules ───────────────────────────────────────────────────────────
        self.keyframes   = KeyframeTracker(kf_dist, kf_yaw)
        self.parking_mgr = ParkingSpaceManager()
        self.img_buf     = ImageBuffer()
        self.vlm         = VLMClient(vlm_server)
        self.executor    = NavigationExecutor(
            st_rate, arc_interp_step, arc_execute_rate, arc_wp_index
        )
        self.hybrid_a    = HybridAStarInterface(ha_timeout)
        self.collision   = CollisionMonitor()
        self.evaluator   = PerformanceEvaluator(cs_thresh)

        # TF for final pose readback (camera_init → map)
        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf)

        self._task_start_time = None

    # ── Utility ───────────────────────────────────────────────────────────────

    def _odom_to_vlm_pose(self, odom):
        x   = odom.pose.pose.position.x
        y   = odom.pose.pose.position.y
        q   = odom.pose.pose.orientation
        yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
        return {'x': x, 'y': y, 'yaw': math.degrees(yaw)}

    def _wait_sensors(self, timeout=60.0):
        rospy.loginfo("Waiting for sensors to be ready...")
        deadline = time.time() + timeout
        while time.time() < deadline:
            if rospy.is_shutdown():
                return False
            ok_odom    = self.keyframes.ready()
            ok_images  = self.img_buf.ready()
            ok_parking = self.parking_mgr.ready()
            rospy.loginfo_throttle(5.0,
                "Sensors: odom=%s images=%s parking=%s",
                ok_odom, ok_images, ok_parking
            )
            if ok_odom and ok_images and ok_parking:
                rospy.loginfo("All sensors ready.")
                return True
            time.sleep(0.5)
        rospy.logwarn("Sensor wait timed out after %.1fs", timeout)
        return False

    def _get_final_pose_map(self):
        """Get current ego pose in map frame via TF."""
        odom = self.keyframes.get_curr_odom()
        if odom is None:
            return None
        try:
            ps = PoseStamped()
            ps.header.stamp    = rospy.Time.now()
            ps.header.frame_id = 'camera_init'
            ps.pose            = odom.pose.pose
            tf_ps = self._tf_buf.transform(ps, 'map', rospy.Duration(2.0))
            q = tf_ps.pose.orientation
            yaw = _quat_to_yaw(q.x, q.y, q.z, q.w)
            return (tf_ps.pose.position.x, tf_ps.pose.position.y, yaw)
        except Exception as e:
            rospy.logwarn("Could not get final pose in map frame: %s", e)
            return None

    def _is_timed_out(self):
        if self._task_start_time is None:
            return False
        return (time.time() - self._task_start_time) > self.task_timeout

    # ── Main ─────────────────────────────────────────────────────────────────

    def run(self):
        self._task_start_time = time.time()
        parking_attempted = False
        had_timeout       = False

        # Wait for sensors
        if not self._wait_sensors(timeout=60.0):
            rospy.logwarn("Sensors not ready — aborting task")
            self._finish(parking_attempted=False, timeout=True)
            return

        rospy.loginfo("Starting VLM navigation loop (max %d steps)", self.max_steps)

        for step in range(self.max_steps):
            if rospy.is_shutdown():
                break

            # ── Abort checks ─────────────────────────────────────────────────
            if self.collision.had_collision():
                rospy.logwarn("Collision detected — aborting task at step %d", step)
                self._finish(parking_attempted=parking_attempted, timeout=False)
                return

            if self._is_timed_out():
                rospy.logwarn("Task timeout (%.0fs) — aborting at step %d",
                              self.task_timeout, step)
                had_timeout = True
                self._finish(parking_attempted=parking_attempted, timeout=True)
                return

            # ── Snapshot current state ────────────────────────────────────────
            curr_odom = self.keyframes.get_curr_odom()
            if curr_odom is None:
                rospy.logwarn("No odometry yet — waiting...")
                time.sleep(1.0)
                continue

            history_poses = self.keyframes.get_history()
            curr_pose     = self._odom_to_vlm_pose(curr_odom)
            parking_slots = self.parking_mgr.get_all_slots_body(curr_odom)
            images        = self.img_buf.get_all()

            if images is None:
                rospy.logwarn("Images not ready — waiting...")
                time.sleep(1.0)
                continue

            rospy.loginfo(
                "Step %d | history=%d | slots=%d | pos=(%.2f,%.2f)",
                step, len(history_poses), len(parking_slots),
                curr_pose['x'], curr_pose['y']
            )

            # ── Call VLM ──────────────────────────────────────────────────────
            rospy.loginfo("=======================================================")
            rospy.loginfo("[VLM INPUT] instruction: %s", self.instruction)
            rospy.loginfo("[VLM INPUT] history_poses: %s", history_poses)
            rospy.loginfo("[VLM INPUT] curr_pose: %s", curr_pose)
            rospy.loginfo("[VLM INPUT] parking_slots: %s", parking_slots)
            
            try:
                decision_id, trajectory = self.vlm.predict(
                    self.instruction, history_poses, curr_pose, parking_slots, images
                )
            except Exception as e:
                rospy.logerr("VLM request failed: %s", e)
                time.sleep(2.0)
                continue

            rospy.loginfo("VLM decision_id=%d | trajectory points=%d",
                          decision_id, len(trajectory))

            # ── Execute decision ───────────────────────────────────────────────
            if decision_id > 0:
                # ── PARK ──────────────────────────────────────────────────────
                rospy.loginfo("Parking decision: space id=%d", decision_id)
                parking_attempted = True

                goal_space = self.parking_mgr.get_space_by_id(decision_id)
                if goal_space is None:
                    rospy.logwarn(
                        "Space id=%d not found in accumulated spaces — skipping",
                        decision_id
                    )
                    # Treat as explore step and continue
                    collided = self.executor.arc_navigate(
                        trajectory, curr_odom,
                        self.collision.had_collision
                    )
                    if collided:
                        rospy.logwarn("Collision during fallback exploration — task failed")
                        self._finish(parking_attempted=parking_attempted, timeout=False)
                        return
                    continue

                try:
                    path = self.hybrid_a.request_path(curr_odom, goal_space)
                except RuntimeError as e:
                    rospy.logerr("Hybrid A* failed: %s", e)
                    self._finish(parking_attempted=True, timeout=False)
                    return

                collided = self.executor.park_navigate(
                    path, self.collision.had_collision
                )
                if collided:
                    rospy.logwarn("Collision during parking — task failed")
                    self._finish(parking_attempted=True, timeout=False)
                    return

                rospy.loginfo("Parking complete — evaluating performance")
                self._finish(parking_attempted=True, timeout=False)
                return

            else:
                # ── EXPLORE ───────────────────────────────────────────────────
                collided = self.executor.arc_navigate(
                    trajectory, curr_odom,
                    self.collision.had_collision
                )
                if collided:
                    rospy.logwarn("Collision during exploration at step %d — task failed", step)
                    self._finish(parking_attempted=parking_attempted, timeout=False)
                    return

        # Exhausted max steps
        rospy.logwarn("Reached max VLM steps (%d) without parking", self.max_steps)
        self._finish(parking_attempted=False, timeout=True)

    def _finish(self, parking_attempted, timeout):
        final_pose = self._get_final_pose_map()
        actual_xy  = self.keyframes.get_trajectory_xy()

        self.evaluator.compute_and_save(
            task_dir          = self.task_dir,
            gt_df             = self.gt_df,
            actual_xy         = actual_xy,
            final_pose_map    = final_pose,
            had_collision     = self.collision.had_collision(),
            parking_attempted = parking_attempted,
            timeout           = timeout,
        )
        rospy.loginfo("Task finished. Shutting down node.")
        rospy.signal_shutdown("Task complete")


# ─────────────────────────────────────────────────────────────────────────────

def main():
    node = ClosedLoopTestNode()
    node.run()
    rospy.spin()


if __name__ == '__main__':
    main()

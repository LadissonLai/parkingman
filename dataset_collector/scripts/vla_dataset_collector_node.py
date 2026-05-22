#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
VLA Dataset Collector Node

Subscribes to CARLA camera / odometry / parking-space topics and records
keyframe data at 2 Hz.  Saves images, odom.csv, parking_slots.jsonl and
decision.txt under a timestamped folder.

Usage (example):
  rosrun dataset_collector vla_dataset_collector_node.py \
    _config_file:=/home/u20/codes/LLM_ws/src/LLMParking/dataset_collector/config/instruct.txt \
    _output_base:=/home/user/vla_dataset
"""

import os
import sys
import csv
import json
import math
import threading
import datetime

import rospy
import rospkg
import cv2
import numpy as np
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
import tf
import tf.transformations

try:
    from parking_space_msgs.msg import ParkingSpaceArray
except ImportError:
    print("[ERROR] parking_space_msgs not found – build the workspace first.")
    sys.exit(1)


# ─────────────────────────────  helpers  ────────────────────────────────── #

def quat_to_yaw(q):
    """Return yaw (radians) from a geometry_msgs/Quaternion."""
    return tf.transformations.euler_from_quaternion(
        [q.x, q.y, q.z, q.w])[2]


def normalize_deg(deg):
    """Wrap angle into (-180, 180]."""
    deg = deg % 360.0
    if deg > 180.0:
        deg -= 360.0
    return deg


# ────────────────────────────  collector  ───────────────────────────────── #

class VLADatasetCollector:

    CAMS = ('front', 'rear', 'left', 'right')
    CAM_TOPICS = {
        'front': '/carla/ego_vehicle/rgb_front/image',
        'rear':  '/carla/ego_vehicle/rgb_rear/image',
        'left':  '/carla/ego_vehicle/rgb_left/image',
        'right': '/carla/ego_vehicle/rgb_right/image',
    }

    def __init__(self):
        rospy.init_node('vla_dataset_collector', anonymous=False)

        self._bridge = CvBridge()
        self._tf     = tf.TransformListener()
        self._lock   = threading.Lock()

        # ── Latest sensor snapshots ──────────────────────────────────────
        self._images  = {c: None for c in self.CAMS}
        self._odom    = None
        self._parking = None

        # ── Session state ────────────────────────────────────────────────
        self._known_ids:    set  = set()   # all parking IDs ever received
        self._recording:    bool = False
        self._busy:         bool = False   # guard re-entrant timer invocation
        self._frame_count:  int  = 0
        self._last_kf_pos        = None    # (x, y) of last saved keyframe
        self._last_kf_yaw        = None    # yaw (rad) of last saved keyframe
        self._start_pose         = None    # (x, y, yaw_rad) at recording start
        self._output_dir:   str  = ""
        self._instruction:  str  = ""

        # ── Accumulated data buffers ─────────────────────────────────────
        self._odom_rows:    list = []
        self._parking_rows: list = []

        # ── ROS subscriptions ────────────────────────────────────────────
        for cam, topic in self.CAM_TOPICS.items():
            rospy.Subscriber(topic, Image, self._img_cb(cam), queue_size=2)

        rospy.Subscriber('/carla/ego_vehicle/odometry',
                         Odometry, self._odom_cb, queue_size=20)
        rospy.Subscriber('/parking_map/confirmed_spaces_in_world',
                         ParkingSpaceArray, self._parking_cb, queue_size=10)

        # ── 2 Hz recording timer ─────────────────────────────────────────
        rospy.Timer(rospy.Duration(0.5), self._tick)

        rospy.loginfo("VLADatasetCollector initialised.")

    # ── ROS callbacks ────────────────────────────────────────────────────

    def _img_cb(self, cam):
        def _cb(msg):
            with self._lock:
                self._images[cam] = msg
        return _cb

    def _odom_cb(self, msg):
        with self._lock:
            self._odom = msg

    def _parking_cb(self, msg):
        with self._lock:
            self._parking = msg
            for sp in msg.spaces:
                self._known_ids.add(int(sp.id))

    # ── 2-Hz tick ────────────────────────────────────────────────────────

    def _tick(self, _event):
        if not self._recording or self._busy:
            return

        with self._lock:
            odom    = self._odom
            parking = self._parking
            imgs    = dict(self._images)

        if odom is None:
            return

        cx  = odom.pose.pose.position.x
        cy  = odom.pose.pose.position.y
        cyaw = quat_to_yaw(odom.pose.pose.orientation)

        if not self._is_keyframe(cx, cy, cyaw):
            return

        self._busy        = True
        self._last_kf_pos = (cx, cy)
        self._last_kf_yaw = cyaw
        self._frame_count += 1
        fnum = self._frame_count

        try:
            self._record_odom(odom)
            self._record_parking(parking)
            self._save_images(imgs, fnum)
            rospy.loginfo("KF %06d  pos=(%.2f, %.2f)", fnum, cx, cy)
        except Exception as exc:
            rospy.logwarn("Error while saving KF %d: %s", fnum, exc)
        finally:
            self._busy = False

    # ── keyframe test ─────────────────────────────────────────────────────

    def _is_keyframe(self, x, y, yaw):
        if self._last_kf_pos is None:
            return True
        lx, ly = self._last_kf_pos
        dist = math.hypot(x - lx, y - ly)
        yaw_diff = abs(yaw - self._last_kf_yaw)
        if yaw_diff > math.pi:
            yaw_diff = 2 * math.pi - yaw_diff
        return dist > 0.1 or yaw_diff > math.radians(5.0)

    # ── per-keyframe recorders ────────────────────────────────────────────

    def _record_odom(self, odom):
        x   = odom.pose.pose.position.x
        y   = odom.pose.pose.position.y
        yaw = quat_to_yaw(odom.pose.pose.orientation)
        x_s, y_s, yaw_s = self._to_start_frame(x, y, yaw)
        vx  = odom.twist.twist.linear.x
        vy  = odom.twist.twist.linear.y
        self._odom_rows.append([
            round(x_s,  6),
            round(y_s,  6),
            round(normalize_deg(math.degrees(yaw_s)), 4),
            round(vx,   6),
            round(vy,   6),
        ])

    def _record_parking(self, parking_msg):
        spaces = self._parking_to_ego(parking_msg)
        self._parking_rows.append(spaces)

    def _save_images(self, imgs, fnum):
        fname = f"{fnum:06d}.png"
        for cam in self.CAMS:
            msg = imgs.get(cam)
            if msg is None:
                continue
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                path   = os.path.join(self._output_dir, 'images', cam, fname)
                cv2.imwrite(path, cv_img)
            except Exception as exc:
                rospy.logwarn("Image save error (%s): %s", cam, exc)

    # ── coordinate transforms ─────────────────────────────────────────────

    def _to_start_frame(self, x, y, yaw):
        """Transform map-frame (x, y, yaw) into the start frame."""
        sx, sy, syaw = self._start_pose
        dx, dy = x - sx, y - sy
        c = math.cos(-syaw)
        s = math.sin(-syaw)
        return dx * c - dy * s, dx * s + dy * c, yaw - syaw

    def _parking_to_ego(self, parking_msg):
        """
        Transform all spaces from the global map frame into ego_vehicle frame.

        Returns:
            list[dict]  – transformed space dicts (may be empty list)
            None        – TF lookup failed
        """
        if parking_msg is None or not parking_msg.spaces:
            return []

        try:
            self._tf.waitForTransform(
                'ego_vehicle', 'map', rospy.Time(0), rospy.Duration(1.0))
            trans, rot = self._tf.lookupTransform(
                'ego_vehicle', 'map', rospy.Time(0))
        except Exception as exc:
            rospy.logwarn("TF error (ego_vehicle <- map): %s", exc)
            return None

        R           = np.array(tf.transformations.quaternion_matrix(rot)[:3, :3])
        t           = np.array(trans)
        map_ego_yaw = tf.transformations.euler_from_quaternion(rot)[2]

        result = []
        for sp in parking_msg.spaces:
            p_map = np.array([sp.pose.position.x,
                              sp.pose.position.y,
                              sp.pose.position.z])
            p_ego = R @ p_map + t

            yaw_map = quat_to_yaw(sp.pose.orientation)
            yaw_deg = normalize_deg(math.degrees(yaw_map + map_ego_yaw))

            # Original message:  width  = along arrow,  height = perp to arrow
            # Required in save:  width  = perp to heading, height = along heading
            result.append({
                'id':     int(sp.id),
                'x':      round(float(p_ego[0]), 4),
                'y':      round(float(p_ego[1]), 4),
                'yaw':    round(yaw_deg, 2),
                'width':  round(float(sp.height), 4),   # perp to arrow
                'height': round(float(sp.width),  4),   # along arrow
            })
        return result

    # ── recording lifecycle ───────────────────────────────────────────────

    def start_recording(self):
        """Initialise output directory and recording state."""
        with self._lock:
            odom = self._odom
        if odom is None:
            rospy.logerr("No odometry received yet – cannot start recording.")
            return False

        now = datetime.datetime.now()
        output_base = rospy.get_param(
            '~output_base', '/home/u20/codes/LLM_ws/src/LLMParking/dataset_collector/dataset/ParkingVLA')
        self._output_dir = os.path.join(
            output_base, now.strftime('%Y-%m-%d-%H-%M'))

        for cam in self.CAMS:
            os.makedirs(os.path.join(self._output_dir, 'images', cam),
                        exist_ok=True)

        x   = odom.pose.pose.position.x
        y   = odom.pose.pose.position.y
        yaw = quat_to_yaw(odom.pose.pose.orientation)
        self._start_pose   = (x, y, yaw)
        self._frame_count  = 0
        self._last_kf_pos  = None
        self._odom_rows    = []
        self._parking_rows = []
        self._recording    = True

        rospy.loginfo("Recording started → %s", self._output_dir)
        return True

    def stop_recording(self):
        self._recording = False
        rospy.loginfo("Recording stopped. %d keyframes collected.",
                      self._frame_count)

    def finalize(self, parking_id: int):
        """Flush all buffered data to disk."""

        # odom.csv
        odom_path = os.path.join(self._output_dir, 'odom.csv')
        with open(odom_path, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['x', 'y', 'yaw', 'velocity_x', 'velocity_y'])
            w.writerows(self._odom_rows)

        # parking_slots.txt  (one JSON per line; {} when no spaces)
        slots_txt_path = os.path.join(self._output_dir, 'parking_slots.txt')
        with open(slots_txt_path, 'w', encoding='utf-8') as fh:
            for spaces in self._parking_rows:
                if not spaces:          # None (TF error) or empty list
                    fh.write('{}\n')
                else:
                    fh.write(json.dumps(spaces, ensure_ascii=False) + '\n')

        # decision.txt
        dec_path = os.path.join(self._output_dir, 'decision.txt')
        with open(dec_path, 'w', encoding='utf-8') as fh:
            fh.write(self._instruction + '\n')
            fh.write(str(parking_id) + '\n')

        rospy.loginfo("All data written to %s", self._output_dir)

    # ── interactive run loop ──────────────────────────────────────────────

    def _load_instruction(self):
        cfg = rospy.get_param('~config_file', '/home/u20/codes/LLM_ws/src/LLMParking/dataset_collector/config/instruct.txt')
        if not os.path.isabs(cfg):
            cfg = os.path.join(os.getcwd(), cfg)
        try:
            with open(cfg, 'r', encoding='utf-8') as fh:
                self._instruction = fh.read().strip()
            print(f"[INFO] Instruction loaded: {self._instruction}")
        except FileNotFoundError:
            rospy.logwarn("Config file '%s' not found; instruction will be empty.", cfg)
            self._instruction = ""

    def run(self):
        self._load_instruction()

        print("\n" + "=" * 60)
        print("VLA Dataset Collector")
        print(f"Instruction : {self._instruction or '(none)'}")
        print("=" * 60)

        # ── Step 1: wait for user to press '1' to start ──────────────────
        print("Type '1' and press Enter to START recording.")
        while not rospy.is_shutdown():
            try:
                key = input("> ").strip()
            except EOFError:
                return
            if key == '1':
                if self.start_recording():
                    break
                else:
                    print("[WARN] Waiting for odometry – please try again.")

        if rospy.is_shutdown():
            return

        # ── Step 2: record until Enter ───────────────────────────────────
        print("Recording…  Press Enter to STOP.")
        try:
            input()
        except EOFError:
            pass

        self.stop_recording()

        # ── Step 3: ask for the chosen parking-space ID ──────────────────
        while not rospy.is_shutdown():
            print(f"\nKnown parking space IDs: {sorted(self._known_ids)}")
            raw = input("Enter the chosen parking space ID: ").strip()
            try:
                pid = int(raw)
            except ValueError:
                print("[WARN] Please enter an integer.")
                continue
            if pid in self._known_ids:
                break
            print(f"[WARN] ID {pid} was never seen in the topic. Try again.")

        # ── Step 4: flush to disk ─────────────────────────────────────────
        self.finalize(pid)
        print(f"\nDataset saved to: {self._output_dir}")
        rospy.signal_shutdown("Dataset collection complete.")


# ─────────────────────────────  entry point  ────────────────────────────── #

def main():
    collector = VLADatasetCollector()

    # Spin ROS callbacks in a background daemon thread so the main thread
    # can handle blocking input() calls.
    spin_thread = threading.Thread(target=rospy.spin, daemon=True)
    spin_thread.start()

    collector.run()

    spin_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()

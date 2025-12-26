#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import yaml
import os
import message_filters
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class BevHardStitcherCorrected:
    def __init__(self):
        rospy.init_node('bev_hard_stitcher_node', anonymous=True)
        self.bridge = CvBridge()
        self._load_params()
        self._load_homographies()
        self._create_hard_stitch_masks()  # 核心逻辑在这里

        subs = [message_filters.Subscriber(topic, Image) for topic in self.image_topics.values()]
        self.ts = message_filters.ApproximateTimeSynchronizer(subs, queue_size=10, slop=0.2)
        self.ts.registerCallback(self.image_callback)

        self.bev_pub = rospy.Publisher('/bev/image_stitched_hard', Image, queue_size=1)
        rospy.loginfo("BEV Hard-Stitcher node (Corrected) initialized.")

    def _load_params(self):
        self.camera_positions = ['front', 'rear', 'left', 'right']
        config_dir = rospy.get_param('~config_dir', os.path.join(os.path.dirname(__file__)))
        
        self.homography_paths = {pos: os.path.join(config_dir, f'{pos}_homography.yaml') for pos in self.camera_positions}
        self.image_topics = {pos: f'/carla/ego_vehicle/rgb_{pos}/image' for pos in self.camera_positions}
        
        try:
            with open(self.homography_paths['front'], 'r') as f:
                config = yaml.safe_load(f)
                self.bev_width = int(config['bev_width'])
                self.bev_height = int(config['bev_height'])
        except Exception as e:
            rospy.logerr(f"Failed to load BEV dimensions: {e}"); rospy.signal_shutdown("Config load error.")

    def _load_homographies(self):
        self.homographies = {}
        for pos in self.camera_positions:
            try:
                with open(self.homography_paths[pos], 'r') as f:
                    config = yaml.safe_load(f)
                    self.homographies[pos] = np.array(config['homography_matrix'])
            except Exception as e:
                rospy.logerr(f"Failed to load homography for '{pos}': {e}"); rospy.signal_shutdown("Homography load error.")

    def _create_hard_stitch_masks(self):
        """按照您的修正要求，使用AND运算创建硬拼接掩码。"""
        rospy.loginfo("Creating hard-stitch masks with AND logic...")
        h, w = self.bev_height, self.bev_width
        center = (w // 2, h // 2)

        # --- 步骤 1: 创建对角线分割掩码 ---
        diagonal_masks = {}
        top_left, top_right = (0, 0), (w, 0)
        bottom_left, bottom_right = (0, h), (w, h)

        pts_f = np.array([top_left, top_right, center], dtype=np.int32)
        diagonal_masks['front'] = cv2.fillPoly(np.zeros((h, w), dtype=np.uint8), [pts_f], 255)

        pts_r = np.array([bottom_left, bottom_right, center], dtype=np.int32)
        diagonal_masks['rear'] = cv2.fillPoly(np.zeros((h, w), dtype=np.uint8), [pts_r], 255)
        
        pts_l = np.array([top_left, bottom_left, center], dtype=np.int32)
        diagonal_masks['left'] = cv2.fillPoly(np.zeros((h, w), dtype=np.uint8), [pts_l], 255)

        pts_ri = np.array([top_right, bottom_right, center], dtype=np.int32)
        diagonal_masks['right'] = cv2.fillPoly(np.zeros((h, w), dtype=np.uint8), [pts_ri], 255)
        
        # --- 步骤 2: 创建基础的ROI掩码 (用于去除非地面投影) ---
        # thickness=-1 表示填充整个矩形，这是正确的
        roi_masks = {}
        roi_masks['front'] = cv2.rectangle(np.zeros((h, w), dtype=np.uint8), (0, 0), (w, h//2-10), 255, -1)
        roi_masks['rear']  = cv2.rectangle(np.zeros((h, w), dtype=np.uint8), (0, h//2+18), (w, h), 255, -1)
        roi_masks['left']  = cv2.rectangle(np.zeros((h, w), dtype=np.uint8), (0, 0), (w//2, h), 255, -1)
        roi_masks['right'] = cv2.rectangle(np.zeros((h, w), dtype=np.uint8), (w//2, 0), (w, h), 255, -1)
        
        # --- 步骤 3: 使用AND运算合并掩码，取公共部分 ---
        self.final_masks = {}
        for pos in self.camera_positions:
            # *** 关键修正：使用 bitwise_and ***
            self.final_masks[pos] = cv2.bitwise_and(diagonal_masks[pos], roi_masks[pos])
        
        rospy.loginfo("Hard-stitch masks created successfully using intersection logic.")

    def image_callback(self, front_msg, rear_msg, left_msg, right_msg):
        try:
            images = {
                'front': self.bridge.imgmsg_to_cv2(front_msg, "bgr8"),
                'rear':  self.bridge.imgmsg_to_cv2(rear_msg, "bgr8"),
                'left':  self.bridge.imgmsg_to_cv2(left_msg, "bgr8"),
                'right': self.bridge.imgmsg_to_cv2(right_msg, "bgr8")
            }
        except Exception as e:
            rospy.logerr(f"CvBridge Error: {e}"); return

        warped_images = {pos: cv2.warpPerspective(images[pos], self.homographies[pos], (self.bev_width, self.bev_height)) for pos in self.camera_positions}

        # --- 步骤 4: 应用掩码并合成最终图像 ---
        final_bev = np.zeros((self.bev_height, self.bev_width, 3), dtype=np.uint8)
        for pos in self.camera_positions:
            cutout = cv2.bitwise_and(warped_images[pos], warped_images[pos], mask=self.final_masks[pos])
            final_bev = cv2.add(final_bev, cutout)

        try:
            bev_msg = self.bridge.cv2_to_imgmsg(final_bev, "bgr8")
            bev_msg.header.stamp = rospy.Time.now()
            bev_msg.header.frame_id = "bev_frame"
            self.bev_pub.publish(bev_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing final BEV image: {e}")

if __name__ == '__main__':
    try:
        BevHardStitcherCorrected()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
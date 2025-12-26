#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
import yaml
import os
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class GenericBevWarper:
    def __init__(self):
        rospy.init_node('generic_bev_warper', anonymous=True)
        self._load_params()
        self.bridge = CvBridge()
        self.homography_matrix = None
        self.mask = None # <--- 新增: 用于存储掩码
        
        self._load_config_from_yaml()
        self._create_bev_mask() # <--- 新增: 创建掩码

        if self.homography_matrix is None:
            rospy.signal_shutdown("Homography not loaded.")
            return

        self.bev_pub = rospy.Publisher(self.bev_topic, Image, queue_size=1)
        self.rgb_sub = rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1)
        rospy.loginfo("Generic BEV Warper node started and configured.")
        self._log_params()

    def _load_params(self):
        # camera_position 参数对于创建正确的掩码至关重要
        self.camera_position = rospy.get_param('~camera/position', 'left') 
        default_h_path = os.path.join(os.path.dirname(__file__), f'{self.camera_position}_homography.yaml')
        self.homography_path = rospy.get_param('~homography_path', default_h_path)
        self.image_topic = rospy.get_param('~image_topic', f'/carla/ego_vehicle/rgb_{self.camera_position}/image')
        self.bev_topic = rospy.get_param('~bev_topic', f'/bev/image_{self.camera_position}')
        
        self.bev_range_fallback = rospy.get_param('~bev/range', 10.0)
        self.bev_resolution_fallback = rospy.get_param('~bev/resolution', 0.05)
        self.bev_width = None
        self.bev_height = None

    def _load_config_from_yaml(self):
        try:
            with open(self.homography_path, 'r') as f:
                config = yaml.safe_load(f)
                self.homography_matrix = np.array(config['homography_matrix'])
                self.bev_width = int(config['bev_width'])
                self.bev_height = int(config['bev_height'])
        except Exception as e:
            rospy.logwarn(f"Could not load config from YAML, falling back to ROS params. Error: {e}")
            # ... (fallback logic) ...
            
    def _create_bev_mask(self):
        """ 根据相机位置，为BEV图像创建一个只显示有效区域的掩码 """
        # 创建一个全黑的掩码
        self.mask = np.zeros((self.bev_height, self.bev_width), dtype=np.uint8)
        
        # 根据相机位置，定义有效区域 (ROI - Region of Interest)
        # BEV图像中心是 (height/2, width/2)
        h, w = self.bev_height, self.bev_width
        
        if self.camera_position == 'front':
            # 前方区域是图像的上半部分
            # 为了融合，通常会多包含一些中心区域
            roi = [(0, 0), (w, h // 2-20)] # 多出来的50像素用于融合
        elif self.camera_position == 'rear':
            # 后方区域是图像的下半部分
            roi = [(0, h // 2 + 28 ), (w, h)]
        elif self.camera_position == 'left':
            # 左方区域是图像的左半部分
            roi = [(0, 0), (w // 2, h)]
        elif self.camera_position == 'right':
            # 右方区域是图像的右半部分
            roi = [(w // 2, 0), (w, h)]
        else:
            rospy.logwarn("Invalid camera_position for mask. Defaulting to full image.")
            self.mask.fill(255) # 如果位置无效，则显示全部
            return
        
        # 在掩码的有效区域内填充白色 (255)
        cv2.rectangle(self.mask, roi[0], roi[1], 255, -1)
        rospy.loginfo(f"Created BEV mask for '{self.camera_position}' camera.")


    def _log_params(self):
        rospy.loginfo(f"--- Warper Configuration for '{self.camera_position}' ---")
        # ... (log messages) ...

    def image_callback(self, rgb_msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e); return

        bev_output_size = (self.bev_width, self.bev_height)
        
        # 1. 正常进行透视变换，得到一个包含“鬼影”的完整BEV图
        bev_image_full = cv2.warpPerspective(
            src=rgb_image,
            M=self.homography_matrix,
            dsize=bev_output_size,
            flags=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0)
        )
        
        # 2. 使用按位与操作，将掩码应用到BEV图上
        #    只有掩码中为白色(255)的区域的像素才会被保留
        bev_image_masked = cv2.bitwise_and(bev_image_full, bev_image_full, mask=self.mask)

        try:
            # 发布被掩码裁剪后的图像
            bev_msg = self.bridge.cv2_to_imgmsg(bev_image_masked, "bgr8")
            bev_msg.header = rgb_msg.header
            bev_msg.header.frame_id = "bev_frame"
            self.bev_pub.publish(bev_msg)
        except CvBridgeError as e:
            rospy.logerr(e)

if __name__ == '__main__':
    try:
        GenericBevWarper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
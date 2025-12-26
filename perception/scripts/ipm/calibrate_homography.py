#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tf_trans
import numpy as np
import cv2
import yaml
import os
from sensor_msgs.msg import CameraInfo

class GenericHomographyCalibrator:
    def __init__(self):
        rospy.init_node('generic_homography_calibrator', anonymous=True)
        self._load_params()
        self.K = None
        self.T_vehicle_to_cam = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("Generic Homography Calibrator initialized.")
        self._log_params()

    def _load_params(self):
        """加载所有参数，并根据 camera_position 动态设置默认值。"""
        # --- 相机位置是基础参数，默认为'front' ---
        self.camera_position = rospy.get_param('~camera/position', 'right')  # 可选值: 'front', 'rear', 'left', 'right'

        # --- BEV参数 ---
        self.bev_range = rospy.get_param('~bev/range', 10.0)
        self.bev_resolution = rospy.get_param('~bev/resolution', 0.05)
        self.bev_width = int(2 * self.bev_range / self.bev_resolution)
        self.bev_height = int(2 * self.bev_range / self.bev_resolution)

        # --- 基于camera_position动态生成默认话题和TF frame ---
        self.camera_info_topic = rospy.get_param('~camera/info_topic', f'/carla/ego_vehicle/rgb_{self.camera_position}/camera_info')
        self.vehicle_frame = rospy.get_param('~tf/vehicle_frame', 'ego_vehicle')
        self.camera_frame = rospy.get_param('~tf/camera_frame', f'ego_vehicle/rgb_{self.camera_position}')
        
        # --- 动态生成默认输出文件名 ---
        default_path = os.path.join(os.path.dirname(__file__), f'{self.camera_position}_homography.yaml')
        self.output_path = rospy.get_param('~output/path', default_path)
        
        # --- 虚拟点生成参数 ---
        self.grid_points_per_side = rospy.get_param('~calibration/grid_points', 10)
        self.min_dist_from_vehicle = rospy.get_param('~calibration/min_dist', 1.0) # 对于侧方相机，1.0米可能更合适

    def _log_params(self):
        rospy.loginfo("--- Calibration Parameters ---")
        rospy.loginfo(f"  Camera Position: {self.camera_position}")
        rospy.loginfo(f"  Camera Info Topic: {self.camera_info_topic}")
        rospy.loginfo(f"  Camera Frame: {self.camera_frame}")
        rospy.loginfo(f"  Output File: {self.output_path}")
        rospy.loginfo(f"  BEV Dimensions: {self.bev_width}x{self.bev_height} px")

    def _get_camera_info(self):
        try:
            cam_info_msg = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=10.0)
            self.K = np.array(cam_info_msg.K).reshape(3, 3)
        except rospy.ROSException as e:
            rospy.logerr(f"Failed to get CameraInfo from '{self.camera_info_topic}': {e}")
            rospy.signal_shutdown("Could not get CameraInfo.")

    def _get_extrinsics(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.camera_frame, self.vehicle_frame, rospy.Time(0), rospy.Duration(5.0))
            t = transform.transform.translation
            q = transform.transform.rotation
            trans_mat = tf_trans.translation_matrix([t.x, t.y, t.z])
            rot_mat = tf_trans.quaternion_matrix([q.x, q.y, q.z, q.w])
            self.T_vehicle_to_cam = np.dot(trans_mat, rot_mat)
        except Exception as e:
            rospy.logerr(f"Failed to get TF transform from '{self.vehicle_frame}' to '{self.camera_frame}': {e}")
            rospy.signal_shutdown("Could not get TF transform.")

    def _create_virtual_ground_points(self):
        """根据相机位置，在车辆坐标系下创建地面虚拟点。"""
        points_3d = []
        gps, min_d, max_d = self.grid_points_per_side, self.min_dist_from_vehicle, self.bev_range
        
        if self.camera_position == 'front':
            # 车辆前方: X为正, Y在车辆两侧
            for x in np.linspace(min_d, max_d, gps):
                for y in np.linspace(-max_d / 2, max_d / 2, gps):
                    points_3d.append([x, y, 0.0])
        
        elif self.camera_position == 'rear':
            # 车辆后方: X为负, Y在车辆两侧
            for x in np.linspace(-max_d, -min_d, gps):
                for y in np.linspace(-max_d / 2, max_d / 2, gps):
                    points_3d.append([x, y, 0.0])
        
        elif self.camera_position == 'left':
            # 车辆左方: Y为正, X在车辆前后
            for y in np.linspace(min_d, max_d, gps):
                for x in np.linspace(-max_d / 2, max_d / 2, gps):
                    points_3d.append([x, y, 0.0])

        elif self.camera_position == 'right':
            # 车辆右方: Y为负, X在车辆前后
            for y in np.linspace(-max_d, -min_d, gps):
                for x in np.linspace(-max_d / 2, max_d / 2, gps):
                    points_3d.append([x, y, 0.0])
        else:
            rospy.logerr(f"Invalid camera_position: '{self.camera_position}'. Use 'front', 'rear', 'left', or 'right'.")
            return None # 返回None以示错误

        return np.array(points_3d, dtype=np.float32)

    def calculate_homography(self):
        self._get_camera_info()
        self._get_extrinsics()

        points_3d_vehicle = self._create_virtual_ground_points()
        
        # 检查虚拟点是否成功创建
        if points_3d_vehicle is None:
            rospy.logwarn(f"Skipping homography calculation for invalid position '{self.camera_position}'.")
            return

        # 计算源点 (相机图像中的2D像素)
        points_3d_vehicle_h = np.hstack([points_3d_vehicle, np.ones((len(points_3d_vehicle), 1))])
        points_3d_cam_h = self.T_vehicle_to_cam @ points_3d_vehicle_h.T
        points_2d_cam_h = self.K @ points_3d_cam_h[:3, :]
        
        valid_mask = points_2d_cam_h[2, :] > 1e-3
        points_2d_cam_h = points_2d_cam_h[:, valid_mask]
        points_3d_vehicle_valid = points_3d_vehicle[valid_mask]

        src_points = (points_2d_cam_h[:2, :] / points_2d_cam_h[2, :]).T
        
        # 计算目标点 (BEV图像中的2D像素)
        x_vehicle = points_3d_vehicle_valid[:, 0]
        y_vehicle = points_3d_vehicle_valid[:, 1]

        pixel_u = (self.bev_width / 2) - y_vehicle / self.bev_resolution
        pixel_v = (self.bev_height / 2) - x_vehicle / self.bev_resolution
        dst_points = np.vstack([pixel_u, pixel_v]).T
        
        # 计算并保存单应性矩阵
        H, _ = cv2.findHomography(src_points, dst_points, cv2.RANSAC, 5.0)
        if H is not None:
            output_dir = os.path.dirname(self.output_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)

            data = {
                'homography_matrix': H.tolist(),
                'bev_width': self.bev_width,
                'bev_height': self.bev_height
            }
            with open(self.output_path, 'w') as f:
                yaml.dump(data, f, default_flow_style=False)
            rospy.loginfo(f"Homography matrix and config saved to {self.output_path}")
        else:
            rospy.logerr("Could not compute homography matrix.")

if __name__ == '__main__':
    try:
        calibrator = GenericHomographyCalibrator()
        rospy.sleep(1.0)
        calibrator.calculate_homography()
    except rospy.ROSInterruptException:
        pass
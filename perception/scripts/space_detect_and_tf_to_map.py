#! /home/u20/miniforge3/envs/yolo-obb/bin/python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import cv2
from ultralytics import YOLO
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray
import math
from parking_space_msgs.msg import ParkingSpace, ParkingSpaceArray # 自定义消息类型

class ParkingSpaceDetector:
    def __init__(self):
        # ... (其他初始化代码保持不变) ...
        rospy.init_node('parking_space_detector', anonymous=True)
        self.bev_resolution = rospy.get_param('~bev_resolution', 0.05)
        self.image_width = rospy.get_param('~image_width', 400)
        self.image_height = rospy.get_param('~image_height', 400)
        self.world_frame = rospy.get_param('~world_frame', 'parking_start_map')
        self.vehicle_frame = rospy.get_param('~vehicle_frame', 'ego_vehicle')
        self.vis_parking_spaces = rospy.get_param('~vis_parking_spaces', True)
        self.space_standard_width = rospy.get_param('~space_standard_width', 5.0)
        self.spaec_standard_height = rospy.get_param('~spaec_standard_height', 3.0) # <--- 修正拼写错误
        self.space_standard_area = self.space_standard_width * self.spaec_standard_height
        model_path = rospy.get_param('~model_path', 'yolo-obb/runs/obb/train3/weights/best.pt')
        self.model = YOLO(model_path)
        rospy.loginfo(f"YOLO-OBB model loaded from {model_path}")
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("TF listener initialized.")
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/bev/image_stitched_hard", Image, self.image_callback, queue_size=1)
        self.marker_pub = rospy.Publisher("/parking_space_markers", MarkerArray, queue_size=10)
        self.parking_data_pub = rospy.Publisher("/parking_spaces_data", ParkingSpaceArray, queue_size=10)
        self.result_pub = rospy.Publisher("/parking_space_detections_image", Image, queue_size=10)
        rospy.loginfo("Parking space detector node is running.")


    def image_callback(self, data):
        rospy.loginfo_throttle(1.0, "Received image.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error: {e}")
            return

        results = self.model(cv_image, verbose=False)

        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, 
                                                        self.vehicle_frame, 
                                                        data.header.stamp, 
                                                        rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"TF transform not found from {self.vehicle_frame} to {self.world_frame}: {e}")
            return

        marker_array = MarkerArray()
        parking_space_array_msg = ParkingSpaceArray()
        parking_space_array_msg.header.stamp = data.header.stamp
        parking_space_array_msg.header.frame_id = self.world_frame

        # 使用 DELETEALL 一次性删除所有旧的Marker
        delete_marker = Marker()
        delete_marker.header.frame_id = self.world_frame
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        marker_id = 0
        
        for result in results:
            if result.obb is None:
                continue
                
            xywhr_data = result.obb.xywhr.cpu().numpy()
            confs_data = result.obb.conf.cpu().numpy()

            for i, row in enumerate(xywhr_data):
                px, py, w_px, h_px, r_rad = row
                confidence = float(confs_data[i])
                if confidence < 0.6:
                    rospy.logwarn_throttle(1.0, f"Skipping low-confidence detection with confidence {confidence:.2f}.")
                    continue
                rospy.loginfo(f"yolo-obb with center ({px}, {py}) with size ({w_px}, {h_px}) and rotation {math.degrees(r_rad):.2f} degree.")
                
                # -- 像素坐标 -> 自车坐标系 (ego_vehicle) --
                x_vehicle = (self.image_height / 2.0 - py) * self.bev_resolution
                y_vehicle = (self.image_width / 2.0 - px) * self.bev_resolution
                width_m = w_px * self.bev_resolution
                height_m = h_px * self.bev_resolution

                # ==============================================================================
                # [核心修正] 调整车辆坐标系下的偏航角
                # ==============================================================================
                # 逻辑修正：如果检测到的盒子“高”大于“宽”，说明它是个竖直车位，
                if height_m > width_m:
                    r_rad -= math.pi / 2.0
                    # 同时，为了让CUBE Marker的尺寸正确匹配，交换宽高
                    width_m, height_m = height_m, width_m
                    
                yaw_vehicle = -r_rad - (math.pi / 2.0)
                
                # 标准化角度到 [-pi, pi] 范围
                while yaw_vehicle > math.pi: 
                    yaw_vehicle -= 2 * math.pi
                while yaw_vehicle < -math.pi: 
                    yaw_vehicle += 2 * math.pi
                # ==============================================================================
                    
                area = width_m * height_m
                if area < self.space_standard_area * 0.8 or area > self.space_standard_area * 1.2:
                    rospy.logwarn_throttle(1.0, f"Skipping parking space with area {area:.2f} m² (expected around {self.space_standard_area:.2f} m²).")
                    continue
                
                pose_vehicle = Pose()
                pose_vehicle.position.x = x_vehicle
                pose_vehicle.position.y = y_vehicle
                pose_vehicle.position.z = 0

                q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw_vehicle)
                pose_vehicle.orientation.x = q[0]
                pose_vehicle.orientation.y = q[1]
                pose_vehicle.orientation.z = q[2]
                pose_vehicle.orientation.w = q[3]

                pose_stamped_vehicle = PoseStamped(header=data.header, pose=pose_vehicle)
                pose_stamped_vehicle.header.frame_id = self.vehicle_frame
                pose_stamped_world = tf2_geometry_msgs.do_transform_pose(pose_stamped_vehicle, transform)

                ps = ParkingSpace()
                ps.id = marker_id
                ps.confidence = confidence
                ps.pose = pose_stamped_world.pose
                ps.width = width_m  # 使用修正后（可能已交换）的宽度
                ps.height = height_m # 使用修正后（可能已交换）的高度
                parking_space_array_msg.spaces.append(ps)

                # --- (可视化) 创建CUBE Marker ---
                marker = Marker()
                marker.header.frame_id = self.world_frame
                marker.header.stamp = data.header.stamp
                marker.ns = "parking_spaces_cubes" # 使用独立的命名空间
                marker.id = marker_id
                marker.type = Marker.CUBE
                marker.action = Marker.ADD
                marker.pose = pose_stamped_world.pose
                marker.scale.x = width_m
                marker.scale.y = height_m
                marker.scale.z = 0.1
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 0.5
                marker.lifetime = rospy.Duration(1.0) # 设置一个生命周期，避免残留
                marker_array.markers.append(marker)
                
                # ==============================================================================
                # [新增] 创建一个 ARROW Marker 来可视化朝向
                # ==============================================================================
                arrow_marker = Marker()
                arrow_marker.header.frame_id = self.world_frame
                arrow_marker.header.stamp = data.header.stamp
                arrow_marker.ns = "parking_spaces_arrows" # 使用独立的命名空间
                arrow_marker.id = marker_id
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.pose = pose_stamped_world.pose # 姿态与CUBE完全相同
                
                # 箭头尺寸：长度2米，宽度0.2米，高度0.2米
                arrow_marker.scale.x = 2.0 
                arrow_marker.scale.y = 0.2
                arrow_marker.scale.z = 0.2
                
                # 箭头颜色：蓝色，不透明
                arrow_marker.color.r = 0.0
                arrow_marker.color.g = 0.5
                arrow_marker.color.b = 1.0
                arrow_marker.color.a = 1.0
                arrow_marker.lifetime = rospy.Duration(1.0) # 设置一个生命周期
                marker_array.markers.append(arrow_marker)
                # ==============================================================================
                
                marker_id += 1

        if parking_space_array_msg.spaces:
            self.parking_data_pub.publish(parking_space_array_msg)
            rospy.loginfo_throttle(1.0, f"Published {len(parking_space_array_msg.spaces)} parking spaces to data topic.")

        if self.vis_parking_spaces and len(marker_array.markers) > 1:
            self.marker_pub.publish(marker_array)

        annotated_image = results[0].plot(conf=True, labels=False)
        try:
            ros_image_result = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            ros_image_result.header = data.header
            self.result_pub.publish(ros_image_result)
        except CvBridgeError as e:
            rospy.logerr(f"CvBridge Error on publishing annotated image: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        detector = ParkingSpaceDetector()
        detector.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Parking space detector node terminated.")
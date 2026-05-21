#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridgeError, CvBridge
import cv2
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import tf_conversions
from geometry_msgs.msg import PoseStamped, Pose, TransformStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import math
import requests
import base64
from parking_space_msgs.msg import ParkingSpace, ParkingSpaceArray # 自定义消息类型

class ParkingSpaceDetectorClient:
    def __init__(self):
        rospy.init_node('parking_space_detector', anonymous=True)
        self.bev_resolution = rospy.get_param('~bev_resolution', 0.003) # 每像素代表的实际距离（单位：米/像素）
        self.image_width = rospy.get_param('~image_width', 1320)
        self.image_height = rospy.get_param('~image_height', 989)
        self.world_frame = rospy.get_param('~world_frame', 'camera_init') # fastlio default
        self.vehicle_frame = rospy.get_param('~vehicle_frame', 'body') # fastlio default
        self.vis_parking_spaces = rospy.get_param('~vis_parking_spaces', True)
        self.space_standard_width = rospy.get_param('~space_standard_width', 0.7) # 车位的标准宽度（单位：米）
        self.space_standard_height = rospy.get_param('~space_standard_height', 0.7) # 车位的标准长度（单位：米）
        self.space_standard_area = self.space_standard_width * self.space_standard_height
        
        # 远程服务端URL
        self.server_url = rospy.get_param('~server_url', 'http://180.85.206.207:9898/detect')
        rospy.loginfo("Using remote YOLO-OBB server at {}".format(self.server_url))

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("TF listener initialized.")

        self.use_odom_fallback = rospy.get_param('~use_odom_fallback', True)
        self.odom_topic = rospy.get_param('~odom_topic', '/Odometry')
        self.latest_odom = None
        if self.use_odom_fallback:
            self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=1)
            rospy.loginfo("Subscribed to Odometry topic for TF fallback: {}".format(self.odom_topic))

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/surround_bev/image", Image, self.image_callback, queue_size=1)
        self.marker_pub = rospy.Publisher("/parking_space_markers", MarkerArray, queue_size=10)
        self.parking_data_pub = rospy.Publisher("/parking_spaces_data", ParkingSpaceArray, queue_size=10)
        self.result_pub = rospy.Publisher("/parking_space_detections_image", Image, queue_size=10)
        rospy.loginfo("Parking space detector client node is running.")

    def odom_callback(self, msg):
        self.latest_odom = msg

    def image_callback(self, data):
        rospy.loginfo_throttle(1.0, "Received image.")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {}".format(e))
            return

        # 编码图片为jpg格式，准备发送给服务端
        _, buffer = cv2.imencode('.jpg', cv_image)
        start_time = rospy.get_time()
        try:
            response = requests.post(self.server_url, files={"file": buffer.tobytes()}, timeout=2.0)
            latency = rospy.get_time() - start_time
            rospy.loginfo_throttle(1.0, "Remote model latency: {:.3f}ms".format(latency * 1000))
            if response.status_code != 200:
                rospy.logerr_throttle(1.0, "Server returned status {}".format(response.status_code))
                return
            result_data = response.json()
        except requests.exceptions.RequestException as e:
            rospy.logerr_throttle(1.0, "Failed to connect to YOLO-OBB server: {}".format(e))
            return

        if self.use_odom_fallback:
            if self.latest_odom is None:
                rospy.logwarn_throttle(1.0, "Waiting for Odometry fallback data on topic '{}'...".format(self.odom_topic))
                return

            # 手动构造一个 TransformStamped 对象作为降级方案
            transform = TransformStamped()
            transform.header.stamp = data.header.stamp
            transform.header.frame_id = self.world_frame
            transform.child_frame_id = self.vehicle_frame
            transform.transform.translation.x = self.latest_odom.pose.pose.position.x
            transform.transform.translation.y = self.latest_odom.pose.pose.position.y
            transform.transform.translation.z = self.latest_odom.pose.pose.position.z
            transform.transform.rotation = self.latest_odom.pose.pose.orientation
        else:
            try:
                transform = self.tf_buffer.lookup_transform(self.world_frame, 
                                                            self.vehicle_frame, 
                                                            data.header.stamp, 
                                                            rospy.Duration(0.1))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn_throttle(1.0, "TF transform not found from {} to {}: {}".format(self.vehicle_frame, self.world_frame, e))
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
        detections = result_data.get("detections", [])
        
        for det in detections:
            px, py, w_px, h_px, r_rad = det["xywhr"]
            confidence = det["conf"]
            
            if confidence < 0.6:
                rospy.logwarn_throttle(1.0, "Skipping low-confidence detection with confidence {:.2f}.".format(confidence))
                continue
            
            # -- 像素坐标 -> 自车坐标系 (ego_vehicle) --
            x_vehicle = (self.image_height / 2.0 - py) * self.bev_resolution
            y_vehicle = (self.image_width / 2.0 - px) * self.bev_resolution
            width_m = w_px * self.bev_resolution
            height_m = h_px * self.bev_resolution

            yaw_vehicle = -r_rad - (math.pi / 2.0)
            
            # 标准化角度到 [-pi, pi] 范围
            while yaw_vehicle > math.pi: 
                yaw_vehicle -= 2 * math.pi
            while yaw_vehicle < -math.pi: 
                yaw_vehicle += 2 * math.pi
                
            area = width_m * height_m
            if area < self.space_standard_area * 0.8 or area > self.space_standard_area * 1.2:
                rospy.logwarn_throttle(1.0, "Skipping parking space with area {:.2f} m² (expected around {:.2f} m²).".format(area, self.space_standard_area))
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
            ps.width = width_m
            ps.height = height_m
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
            arrow_marker.scale.x = 0.2
            arrow_marker.scale.y = 0.1
            arrow_marker.scale.z = 0.1
            
            # 箭头颜色：蓝色，不透明
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.5
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 1.0
            arrow_marker.lifetime = rospy.Duration(1.0) # 设置一个生命周期
            marker_array.markers.append(arrow_marker)
            
            marker_id += 1

        if parking_space_array_msg.spaces:
            self.parking_data_pub.publish(parking_space_array_msg) # 发布到全局世界坐标系下面
            rospy.loginfo_throttle(1.0, "Published {} parking spaces to data topic.".format(len(parking_space_array_msg.spaces)))

        if self.vis_parking_spaces and len(marker_array.markers) > 1:
            self.marker_pub.publish(marker_array)

        # 接收并解码标注好的网络图片
        annotated_image_b64 = result_data.get("annotated_image", "")
        if annotated_image_b64:
            img_bytes = base64.b64decode(annotated_image_b64)
            annotated_image = cv2.imdecode(np.frombuffer(img_bytes, np.uint8), cv2.IMREAD_COLOR)
            try:
                ros_image_result = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                ros_image_result.header = data.header
                self.result_pub.publish(ros_image_result)
            except CvBridgeError as e:
                rospy.logerr("CvBridge Error on publishing annotated image: {}".format(e))

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        client = ParkingSpaceDetectorClient()
        client.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Parking space detector client node terminated.")

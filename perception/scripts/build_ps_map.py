#!/usr/bin/env python

import rospy
import numpy as np
from shapely.geometry import Polygon

# 从你的包中导入自定义消息
# 修改: 导入新的消息类型
from parking_space_msgs.msg import ParkingSpaceArray, ParkingSpace
from llm_perception_msgs.msg import GlobalParkingSpaceDescription, ParkingSpaceDescription

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Point, Pose, PoseStamped
from tf.transformations import euler_from_quaternion # 用于将四元数转换为欧拉角，提取yaw
import tf2_ros
import tf2_geometry_msgs

# 新增：为显示2D文本导入OverlayText消息
from jsk_rviz_plugins.msg import OverlayText


# --- 辅助函数：计算旋转矩形的IOU (保持不变) ---
def calculate_rotated_iou(ps_a, ps_b):
    """
    计算两个ParkingSpace对象（或具有相似pose, width, height属性的对象）的IOU。
    """
    def create_polygon(ps):
        q = [ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w]
        if q[3] == 0 and all(v == 0 for v in q[:3]):
             _, _, yaw = 0, 0, 0
        else:
             _, _, yaw = euler_from_quaternion(q)

        cx, cy = ps.pose.position.x, ps.pose.position.y
        w, h = ps.width, ps.height

        local_corners = [
            (w / 2, h / 2), (w / 2, -h / 2),
            (-w / 2, -h / 2), (-w / 2, h / 2)
        ]
        
        world_corners = []
        cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
        for p_x, p_y in local_corners:
            x = cx + p_x * cos_yaw - p_y * sin_yaw
            y = cy + p_x * sin_yaw + p_y * cos_yaw
            world_corners.append((x, y))
            
        return Polygon(world_corners)

    try:
        poly_a = create_polygon(ps_a)
        poly_b = create_polygon(ps_b)
        
        if not poly_a.is_valid or not poly_b.is_valid:
            rospy.logwarn_throttle(5.0, "IOU calculation failed due to invalid polygon.")
            return 0.0

        intersection_area = poly_a.intersection(poly_b).area
        union_area = poly_a.area + poly_b.area - intersection_area

        if union_area == 0:
            return 0.0
        
        return intersection_area / union_area

    except Exception as e:
        rospy.logerr(f"Error in IOU calculation: {e}")
        return 0.0

class MapParkingSlot:
    def __init__(self, initial_detection, candidate_id):
        self.candidate_id = candidate_id # 内部追踪ID
        self.confirmed_id = -1 # 外部永久ID，-1表示尚未分配
        self.highest_confidence_detection = initial_detection
        self.status = "CANDIDATE"
        self.consecutive_detection_count = 1
        self.last_seen_stamp = rospy.Time.now()

class ParkingMapBuilder:
    def __init__(self):
        rospy.init_node('parking_map_builder', anonymous=True)

        self.gating_distance_threshold = rospy.get_param('~gating_distance_threshold', 5.0)
        self.iou_match_threshold = rospy.get_param('~iou_match_threshold', 0.1)
        self.confirmation_threshold = rospy.get_param('~confirmation_threshold', 10)
        self.publish_rate = rospy.get_param('~publish_rate', 2.0)
        self.cleanup_timeout = rospy.get_param('~cleanup_timeout', 30.0)

        self.map_slots = {}
        # *** 修改：引入两套ID计数器 ***
        self.next_candidate_id = 1 # 用于内部追踪的临时ID
        self.next_confirmed_id = 1 # 用于外部发布的永久ID

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.world_frame = rospy.get_param('~world_frame', 'map')
        self.local_world_frame = rospy.get_param('~local_world_frame', 'parking_start_map')

        self.data_sub = rospy.Subscriber("/parking_spaces_data", ParkingSpaceArray, self.data_callback)
        self.map_marker_pub = rospy.Publisher("/parking_map_markers", MarkerArray, queue_size=10)
        self.overlay_text_pub = rospy.Publisher("/parking_map_info_text", OverlayText, queue_size=10)
        self.confirmed_spaces_pub = rospy.Publisher("/parking_map/confirmed_spaces", ParkingSpaceArray, queue_size=10)
        self.confirmed_spaces_in_world_pub = rospy.Publisher("/parking_map/confirmed_spaces_in_world", ParkingSpaceArray, queue_size=10)
        
        # 新增：用于发布车位ID文本Marker的发布者
        self.text_marker_pub = rospy.Publisher("/perception/parking_space_id_text_markers", MarkerArray, queue_size=10)

        # 新增：用于发布LLM所需格式的全局车位信息的发布者
        self.global_parking_spaces_pub = rospy.Publisher("/perception/global_parking_space_description", GlobalParkingSpaceDescription, queue_size=10)

        # 新增：重置地图的订阅者
        self.reset_sub = rospy.Subscriber("/perception/parking_space_map/reset", Bool, self.reset_map_callback)

        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_map)
        
        rospy.loginfo("Parking Map Builder node is running with updated visualization and ID logic.")

    def reset_map_callback(self, msg):
        if msg.data:
            rospy.logwarn("Resetting parking map state...")
            self.map_slots.clear()
            self.next_candidate_id = 1
            self.next_confirmed_id = 1
            # 立即发布一次以清除可视化
            self.publish_map(None)

    def data_callback(self, msg):
        matched_map_ids = set()
        
        for new_detection in msg.spaces:
            best_match_id, max_iou = -1, -1.0
            candidate_ids = []
            
            # 使用内部追踪ID (字典的键) 进行匹配
            for map_id, map_slot in self.map_slots.items():
                map_pos = map_slot.highest_confidence_detection.pose.position
                new_pos = new_detection.pose.position
                dist = np.linalg.norm([map_pos.x - new_pos.x, map_pos.y - new_pos.y])
                if dist < self.gating_distance_threshold:
                    candidate_ids.append(map_id)
            
            if candidate_ids:
                for map_id in candidate_ids:
                    if map_id in matched_map_ids:
                        continue
                    
                    map_slot = self.map_slots[map_id]
                    iou = calculate_rotated_iou(new_detection, map_slot.highest_confidence_detection)
                    
                    if iou > max_iou:
                        max_iou = iou
                        best_match_id = map_id
            
            if max_iou > self.iou_match_threshold:
                self.update_slot(best_match_id, new_detection)
                matched_map_ids.add(best_match_id)
            else:
                self.create_new_slot(new_detection)

        current_time = rospy.Time.now()
        ids_to_delete = []
        for map_id, map_slot in self.map_slots.items():
            if map_id not in matched_map_ids:
                map_slot.consecutive_detection_count = 0
            
            if map_slot.status == "CANDIDATE" and \
               (current_time - map_slot.last_seen_stamp).to_sec() > self.cleanup_timeout:
                ids_to_delete.append(map_id)
        
        for map_id in ids_to_delete:
            del self.map_slots[map_id]
            rospy.loginfo(f"Removed stale CANDIDATE slot with internal ID {map_id} due to timeout.")

    def update_slot(self, map_id, detection):
        map_slot = self.map_slots[map_id]
        
        map_slot.consecutive_detection_count += 1
        
        # *** 修改：核心逻辑，在状态提升时分配永久ID ***
        if map_slot.status == "CANDIDATE" and map_slot.consecutive_detection_count >= self.confirmation_threshold:
            map_slot.status = "CONFIRMED"
            # 检查是否已分配过永久ID，防止重复分配
            if map_slot.confirmed_id == -1:
                map_slot.confirmed_id = self.next_confirmed_id
                self.next_confirmed_id += 1
                rospy.loginfo(f"Parking slot (internal ID {map_id}) PROMOTED to CONFIRMED with permanent ID {map_slot.confirmed_id}.")
        
        if detection.confidence > map_slot.highest_confidence_detection.confidence:
            map_slot.highest_confidence_detection = detection
            
        map_slot.last_seen_stamp = rospy.Time.now()
    
    def create_new_slot(self, detection):
        # *** 修改：使用 candidate_id 计数器 ***
        new_candidate_id = self.next_candidate_id
        self.map_slots[new_candidate_id] = MapParkingSlot(detection, new_candidate_id)
        rospy.loginfo(f"Created new CANDIDATE parking slot with internal ID {new_candidate_id}.")
        self.next_candidate_id += 1

    def publish_map(self, event=None):
        marker_array = MarkerArray()
        text_marker_array = MarkerArray()  # 新增：用于存储文本Marker
        
        delete_marker = Marker()
        delete_marker.header.frame_id = self.local_world_frame
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        # 同样清空文本 Marker
        delete_text_marker = Marker()
        delete_text_marker.header.frame_id = self.world_frame # 注意这里使用 world_frame
        delete_text_marker.action = Marker.DELETEALL
        text_marker_array.markers.append(delete_text_marker)

        confirmed_count = 0
        
        # 尝试获取变换，如果失败则只发布局部坐标系的可视化
        try:
            transform = self.tf_buffer.lookup_transform(self.world_frame, self.local_world_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn_throttle(1.0, f"TF transform not found from {self.local_world_frame} to {self.world_frame}: {e}")
            transform = None

        # --- 初始化新的全局车位消息数组 ---
        global_parking_spaces_array = GlobalParkingSpaceDescription()
        global_parking_spaces_array.header.stamp = rospy.Time.now()
        global_parking_spaces_array.header.frame_id = self.world_frame
        
        for slot_id, map_slot in self.map_slots.items():
            if map_slot.status != "CONFIRMED":
                continue
            
            confirmed_count += 1
            
            best_det = map_slot.highest_confidence_detection
            pose = best_det.pose
            w = best_det.width
            h = best_det.height
            
            # --- (可视化) 创建CUBE Marker（填充矩形） ---
            cube_marker = Marker()
            cube_marker.header.frame_id = self.local_world_frame
            cube_marker.header.stamp = rospy.Time.now()
            cube_marker.ns = "parking_spaces_cubes"
            cube_marker.id = map_slot.confirmed_id
            cube_marker.type = Marker.CUBE
            cube_marker.action = Marker.ADD
            cube_marker.pose = pose
            cube_marker.scale.x = w
            cube_marker.scale.y = h
            cube_marker.scale.z = 0.1
            cube_marker.color.r = 0.0
            cube_marker.color.g = 1.0
            cube_marker.color.b = 0.0
            cube_marker.color.a = 0.5
            cube_marker.lifetime = rospy.Duration()  # 永久显示
            marker_array.markers.append(cube_marker)
            
            # --- (可视化) 创建ARROW Marker（方向指示） ---
            arrow_marker = Marker()
            arrow_marker.header.frame_id = self.local_world_frame
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = "parking_spaces_arrows"
            arrow_marker.id = map_slot.confirmed_id
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose = pose  # 姿态与CUBE相同
                
            # 箭头尺寸：长度2米，宽度0.2米，高度0.2米
            arrow_marker.scale.x = 2.0 
            arrow_marker.scale.y = 0.2
            arrow_marker.scale.z = 0.2
                
            # 箭头颜色：蓝色，不透明
            arrow_marker.color.r = 0.0
            arrow_marker.color.g = 0.5
            arrow_marker.color.b = 1.0
            arrow_marker.color.a = 1.0
            arrow_marker.lifetime = rospy.Duration()  # 永久显示
            marker_array.markers.append(arrow_marker)

            # --- (可视化) 创建 ID TEXT Marker (全局坐标系) 和 填充 GlobalParkingSpace消息 ---
            if transform:
                local_pose_stpd = PoseStamped(header=rospy.Header(frame_id=self.local_world_frame, stamp=rospy.Time(0)), 
                                              pose=pose)
                try:
                    transformed_ps = tf2_geometry_msgs.do_transform_pose(local_pose_stpd, transform)
                    
                    # --- 为 RViz 创建文本 Marker ---
                    text_marker = Marker()
                    text_marker.header.frame_id = self.world_frame
                    text_marker.header.stamp = rospy.Time.now()
                    text_marker.ns = "parking_spaces_ids"
                    text_marker.id = map_slot.confirmed_id
                    text_marker.type = Marker.TEXT_VIEW_FACING
                    text_marker.action = Marker.ADD
                    
                    # 稍微抬高一点，避免被车位遮挡
                    text_marker.pose.position.x = transformed_ps.pose.position.x
                    text_marker.pose.position.y = transformed_ps.pose.position.y
                    text_marker.pose.position.z = transformed_ps.pose.position.z + 0.5 
                    
                    text_marker.pose.orientation.w = 1.0 # TEXT_VIEW_FACING 不需要旋转，但需要合法的四元数
                    
                    text_marker.text = str(map_slot.confirmed_id)
                    text_marker.scale.z = 1.0 # 字体高度
                    
                    # 红色字体
                    text_marker.color.r = 1.0
                    text_marker.color.g = 0.0
                    text_marker.color.b = 0.0
                    text_marker.color.a = 1.0
                    
                    text_marker.lifetime = rospy.Duration()
                    text_marker_array.markers.append(text_marker)

                    # --- 填充 GlobalParkingSpace 消息 ---
                    global_ps_msg = ParkingSpaceDescription()
                    global_ps_msg.id = map_slot.confirmed_id
                    global_ps_msg.x = transformed_ps.pose.position.x
                    global_ps_msg.y = transformed_ps.pose.position.y

                    # 从四元数提取 Yaw 角并转换为度数 (-180 到 180)
                    orientation_q = [transformed_ps.pose.orientation.x,
                                     transformed_ps.pose.orientation.y,
                                     transformed_ps.pose.orientation.z,
                                     transformed_ps.pose.orientation.w]
                    # euler_from_quaternion 返回 (roll, pitch, yaw) 弧度
                    _, _, yaw_rad = euler_from_quaternion(orientation_q)
                    global_ps_msg.yaw_degrees = np.degrees(yaw_rad)
                    
                    # length 沿着局部x轴 (ParkingSpace.width)
                    global_ps_msg.length = w
                    # width 沿着局部y轴 (ParkingSpace.height)
                    global_ps_msg.width = h

                    global_parking_spaces_array.spaces.append(global_ps_msg)

                except Exception as e:
                    rospy.logwarn_throttle(5.0, f"Failed to transform pose for text marker or GlobalParkingSpace: {e}")

        self.map_marker_pub.publish(marker_array)
        self.text_marker_pub.publish(text_marker_array) # 发布文本 Marker
        
        overlay_text = OverlayText()
        overlay_text.width = 400
        overlay_text.height = 40
        overlay_text.left = 10
        overlay_text.top = 10
        overlay_text.text_size = 12
        overlay_text.font = "Ubuntu"
        overlay_text.bg_color.a = 0.5
        overlay_text.fg_color.r, overlay_text.fg_color.g, overlay_text.fg_color.b, overlay_text.fg_color.a = 1.0, 1.0, 1.0, 1.0
        overlay_text.text = f"Vacant Parking Space Count: {confirmed_count}"
        self.overlay_text_pub.publish(overlay_text)
        
        confirmed_spaces_array = ParkingSpaceArray()
        confirmed_spaces_in_world = ParkingSpaceArray()
        confirmed_spaces_array.header.frame_id = self.local_world_frame
        confirmed_spaces_in_world.header.frame_id = self.world_frame
        confirmed_spaces_array.header.stamp = rospy.Time.now()
        confirmed_spaces_in_world.header.stamp = rospy.Time.now()

        # 如果没有transform，后面的转换循环就无法进行，直接跳过
        if not transform:
            # 即使没有transform，也要发布空的GlobalParkingSpaceArray，避免LLM端因为没有消息而报错
            global_parking_spaces_array.total_count = len(global_parking_spaces_array.spaces)
            self.global_parking_spaces_pub.publish(global_parking_spaces_array)
            return

        for _, map_slot in self.map_slots.items():
            if map_slot.status == "CONFIRMED":
                # *** 修改：发布的消息ID使用永久ID ***
                space_msg = map_slot.highest_confidence_detection
                space_msg.id = map_slot.confirmed_id
                confirmed_spaces_array.spaces.append(space_msg)
                
                local_pose_stpd = PoseStamped(header=rospy.Header(frame_id=self.local_world_frame, stamp=rospy.Time(0)), 
                                              pose=space_msg.pose)
                transformed_ps = tf2_geometry_msgs.do_transform_pose(local_pose_stpd, transform)
                
                ps_inworld_msg = ParkingSpace()
                # *** 修改：发布的消息ID使用永久ID ***
                ps_inworld_msg.id = map_slot.confirmed_id
                ps_inworld_msg.pose = transformed_ps.pose
                ps_inworld_msg.width = space_msg.width
                ps_inworld_msg.height = space_msg.height
                ps_inworld_msg.confidence = space_msg.confidence
                confirmed_spaces_in_world.spaces.append(ps_inworld_msg)

        self.confirmed_spaces_pub.publish(confirmed_spaces_array)
        self.confirmed_spaces_in_world_pub.publish(confirmed_spaces_in_world)
        
        # --- 发布新的全局车位消息数组 ---
        global_parking_spaces_array.total_count = len(global_parking_spaces_array.spaces)
        self.global_parking_spaces_pub.publish(global_parking_spaces_array)


if __name__ == '__main__':
    try:
        ParkingMapBuilder()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Parking Map Builder node terminated.")
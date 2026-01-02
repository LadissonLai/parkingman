#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import carla
import numpy as np
import math
from geometry_msgs.msg import Point, Quaternion, Vector3
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler, euler_from_quaternion, quaternion_matrix

# --- 导入自定义 MSG (请修改为你的实际包名) ---
from llm_perception_msgs.msg import VehicleDescription, GlobalVehicleDescription

# ==========================================
# 1. 颜色配置与辅助函数
# ==========================================

# CARLA车辆的标准颜色库（基于实际分析的RGB值）
# 涵盖CARLA动态生成车辆的所有常见颜色
STANDARD_COLORS = {
    # 基础颜色
    'White':      (1.00, 1.00, 1.00),  # 255,255,255
    'Black':      (0.00, 0.00, 0.00),  # 0,0,0
    'Gray':       (0.50, 0.50, 0.50),  # 128,128,128
    'Silver':     (0.75, 0.75, 0.75),  # 192,192,192

    # 主要颜色
    'Red':        (1.00, 0.00, 0.00),  # 255,0,0
    'Blue':       (0.00, 0.00, 1.00),  # 0,0,255
    'Green':      (0.00, 1.00, 0.00),  # 0,255,0
    'Yellow':     (1.00, 0.73, 0.00),  # 255,255,0

    # 扩展颜色
    'Orange':     (1.0, 0.27, 0.00),  # 255,165,0
    'Purple':     (0.50, 0.00, 0.50),  # 128,0,128
    'Pink':       (1.00, 0.75, 0.80),  # 255,192,203
    'Brown':      (0.65, 0.16, 0.16),  # 165,42,42
    'Cyan':       (0.00, 1.00, 1.00),  # 0,255,255

    # 深色调
    'Dark Blue':  (0.00, 0.00, 0.35),  # 0,0,139
    'Dark Green': (0.00, 0.39, 0.00),  # 0,100,0
    'Dark Red':   (0.55, 0.00, 0.00),  # 139,0,0

    # 浅色调
    'Light Blue': (0.68, 0.85, 0.90),  # 173,216,230
    'Light Green':(0.56, 0.93, 0.56), # 144,238,144
    'Light Blue2':(0.24, 0.34, 0.56),

    # 金属色调
    'Gold':       (1.00, 0.84, 0.00),  # 255,215,0
    'Antique White': (0.98, 0.92, 0.84), # 250,235,215
    'Beige':      (0.96, 0.96, 0.86), # 245,245,220
}

# 颜色语义映射表：将相似颜色归类为更通用的名称
COLOR_SEMANTIC_MAP = {
    # 白色系列 -> 统一为White
    'Antique White': 'White',  # 古董白 -> 白色
    'Beige': 'White',          # 米色 -> 白色

    # 灰色系列 -> 统一为Gray/Silver
    'Silver': 'Gray',      # 浅灰 -> 灰色

    # 蓝色系列 -> 统一为Blue
    'Dark Blue': 'Blue',       # 深蓝 -> 蓝色
    'Light Blue': 'Blue',      # 浅蓝 -> 蓝色
    'Light Blue2': 'Blue',

    # 绿色系列 -> 统一为Green
    'Dark Green': 'Green',     # 深绿 -> 绿色
    'Light Green': 'Green',    # 浅绿 -> 绿色

    # 红色系列 -> 统一为Red
    'Dark Red': 'Red',         # 深红 -> 红色

    # 其他保持不变
    'White': 'White',
    'Black': 'Black',
    'Gray': 'Gray',
    # 'Silver': 'Silver',
    'Red': 'Red',
    'Blue': 'Blue',
    'Green': 'Green',
    'Yellow': 'Yellow',
    'Orange': 'Orange',
    'Purple': 'Purple',
    'Pink': 'Pink',
    'Brown': 'Brown',
    'Cyan': 'Cyan',
    'Gold': 'Gold',
    'Custom': 'Unknown'
}

# 静态车数据库 (你的标定数据)
STATIC_VEHICLE_DB = {
    3183030087427035139:  {'name': 'White', 'rgba': (1.0, 1.0, 1.0, 1.0)},
    4189097565736758488:  {'name': 'Gold',  'rgba': (1.0, 0.84, 0.0, 1.0)},
    6821435750596068981:  {'name': 'Gold',  'rgba': (1.0, 0.84, 0.0, 1.0)},
    7633645211419130701:  {'name': 'Black', 'rgba': (0.1, 0.1, 0.1, 1.0)},
    8510481749745410382:  {'name': 'White', 'rgba': (1.0, 1.0, 1.0, 1.0)},
    8752205197467337684:  {'name': 'Gold',  'rgba': (1.0, 0.84, 0.0, 1.0)},
    12947037574827913776: {'name': 'Gold',  'rgba': (1.0, 0.84, 0.0, 1.0)},
    13506597690829582167: {'name': 'Blue',  'rgba': (0.0, 0.5, 1.0, 1.0)},
    14871698505808869873: {'name': 'Blue',  'rgba': (0.0, 0.5, 1.0, 1.0)},
    15097890229429448875: {'name': 'Black', 'rgba': (0.1, 0.1, 0.1, 1.0)},
    15667345652452848567: {'name': 'White', 'rgba': (1.0, 1.0, 1.0, 1.0)},
    15816635970456745613: {'name': 'White', 'rgba': (1.0, 1.0, 1.0, 1.0)},
    16069205303907037704: {'name': 'White', 'rgba': (1.0, 1.0, 1.0, 1.0)},
    16563077071273106294: {'name': 'Red',   'rgba': (1.0, 0.0, 0.0, 1.0)},
}

def get_closest_color_name(rgb_tuple, similarity_threshold=0.2):
    """
    针对CARLA车辆颜色的优化匹配算法
    CARLA主要使用标准RGB值，因此可以设置更严格的相似度阈值
    包含语义映射，将相似颜色归类为通用名称
    """
    min_dist = float('inf')
    closest_name = 'Custom'

    r1, g1, b1 = rgb_tuple

    for name, (r2, g2, b2) in STANDARD_COLORS.items():
        # 对于CARLA的标准颜色，使用精确匹配
        # CARLA颜色通常是精确的RGB值，不是渐变色
        dist = (r1 - r2)**2 + (g1 - g2)**2 + (b1 - b2)**2

        if dist < min_dist:
            min_dist = dist
            closest_name = name

    # CARLA颜色通常很精确，设置较低的阈值(0.08 ≈ 3.5%误差)
    if min_dist > similarity_threshold:
        return 'Custom'

    # 应用语义映射，将相似颜色归类为通用名称
    return COLOR_SEMANTIC_MAP.get(closest_name, closest_name)

# ==========================================
# 2. 主逻辑类
# ==========================================

class PersistentVehicleMapper:
    def __init__(self):
        rospy.init_node('llm_vehicle_perception', anonymous=True)
        
        # --- 参数 ---
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.host = rospy.get_param('~host', 'localhost')
        self.port = rospy.get_param('~port', 2000)
        self.scan_radius = rospy.get_param('~radius', 60.0)
        self.show_color_text = rospy.get_param('~show_color_text', True)  # 是否显示颜色文字
        
        # --- 状态存储 ---
        self.next_uid = 1
        self.id_map = {}        # CarlaID -> SeqID
        self.memory_vehicles = {} # SeqID -> {Info}
        
        # --- Carla 连接 ---
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(5.0)
            self.world = self.client.get_world()
            rospy.loginfo("Carla 连接成功")
        except Exception as e:
            rospy.logerr(f"Carla 连接失败: {e}")
            return

        # --- 静态资源缓存 ---
        self.static_cache = []
        self.static_locs_numpy = None
        self.label_type = self.resolve_label()
        self.preprocess_static_map()

        # --- Publishers ---
        # 1. 给 RViz 看的可视化 Marker
        self.pub_viz = rospy.Publisher('/perception/global_vehicle_color_id_viz', MarkerArray, queue_size=10)
        
        # 2. 给 LLM 看的结构化文本描述 Msg
        self.pub_llm = rospy.Publisher('/perception/global_vehicle_description', GlobalVehicleDescription, queue_size=5)
        
        # 3. 重置请求订阅
        self.reset_sub = rospy.Subscriber('/perception/vehicle_map/reset', Bool, self.reset_callback)
        self.reset_requested = False

        # 4. 颜色显示开关服务
        self.color_toggle_srv = rospy.Service('/perception/vehicle_map/toggle_color_text', SetBool, self.toggle_color_text_callback)

        rospy.loginfo("节点启动: 正在发布 RViz Marker 和 LLM Description...")
        rospy.loginfo(f"颜色文字显示: {'开启' if self.show_color_text else '关闭'}")

    def resolve_label(self):
        for name in ['Vehicles', 'Car', 'Vehicle']:
            if hasattr(carla.CityObjectLabel, name):
                return getattr(carla.CityObjectLabel, name)
        return 10

    def carla_rot_to_ros_quat(self, rot):
        roll = math.radians(rot.roll)
        pitch = math.radians(-rot.pitch)
        yaw = math.radians(-rot.yaw)
        q = quaternion_from_euler(roll, pitch, yaw)
        return list(q)

    def preprocess_static_map(self):
        rospy.loginfo("构建静态车辆缓存...")
        env_objs = self.world.get_environment_objects(self.label_type)
        locs = []
        for obj in env_objs:
            tf = obj.transform
            bb = obj.bounding_box
            data = {
                'id': obj.id, 'tf': tf, 'bb': bb,
                'pos_arr': np.array([tf.location.x, tf.location.y])
            }
            self.static_cache.append(data)
            locs.append([tf.location.x, tf.location.y])
            
        if locs:
            self.static_locs_numpy = np.array(locs)
        else:
            self.static_locs_numpy = np.empty((0, 2))

    # --- 颜色处理逻辑 ---
    def get_color_info(self, carla_id, actor=None):
        """ 返回 (RGBA_Tuple, Color_Name_String) """
        
        # 1. 如果是静态车，查数据库
        if carla_id in STATIC_VEHICLE_DB:
            entry = STATIC_VEHICLE_DB[carla_id]
            return entry['rgba'], entry['name']
        
        # 2. 如果是动态车，解析属性并匹配最近颜色
        if actor:
            try:
                c_str = actor.attributes.get('color', '200,200,200')
                parts = c_str.split(',')
                if len(parts) == 3:
                    r = int(parts[0]) / 255.0
                    g = int(parts[1]) / 255.0
                    b = int(parts[2]) / 255.0
                    # 匹配最近的颜色名
                    name = get_closest_color_name((r, g, b))
                    return (r, g, b, 1.0), name
            except:
                pass
        
        # 3. 兜底
        return (0.7, 0.7, 0.7, 1.0), "Gray"

    def register_vehicle(self, carla_id, carla_tf, carla_bb, is_static, actor=None):
        seq_id = self.next_uid
        self.next_uid += 1
        self.id_map[carla_id] = seq_id
        
        # 坐标转换
        pos = [carla_tf.location.x, -carla_tf.location.y, carla_tf.location.z]
        quat = self.carla_rot_to_ros_quat(carla_tf.rotation)
        dims = [carla_bb.extent.x*2, carla_bb.extent.y*2, carla_bb.extent.z*2]
        
        # 获取颜色 (数值 + 文本)
        rgba, color_name = self.get_color_info(carla_id, actor)
        
        self.memory_vehicles[seq_id] = {
            'pos': pos, 
            'quat': quat, 
            'dims': dims, 
            'rgba': rgba,
            'color_text': color_name
        }
        print(f"{seq_id}: {color_name} -> {rgba}")

    def update_perception(self):
        actors = self.world.get_actors().filter('vehicle.*')
        ego_actor = None
        for actor in actors:
            if actor.attributes.get('role_name') == self.role_name:
                ego_actor = actor
                break
        if ego_actor is None: return

        ego_loc = ego_actor.get_transform().location
        ego_arr = np.array([ego_loc.x, ego_loc.y])

        # 1. 静态车处理
        if self.static_locs_numpy.shape[0] > 0:
            diff = self.static_locs_numpy - ego_arr
            dist_sq = np.sum(diff**2, axis=1)
            nearby_indices = np.where(dist_sq < self.scan_radius**2)[0]
            
            for idx in nearby_indices:
                obj = self.static_cache[idx]
                if obj['id'] not in self.id_map:
                    self.register_vehicle(obj['id'], obj['tf'], obj['bb'], True)

        # 2. 动态车处理
        for actor in actors:
            if actor.id == ego_actor.id: continue
            loc = actor.get_transform().location
            dist = math.sqrt((loc.x - ego_loc.x)**2 + (loc.y - ego_loc.y)**2)
            if dist < self.scan_radius:
                if actor.id not in self.id_map:
                    self.register_vehicle(actor.id, actor.get_transform(), actor.bounding_box, False, actor)
                else:
                    # 更新位置
                    tf = actor.get_transform()
                    seq_id = self.id_map[actor.id]
                    self.memory_vehicles[seq_id]['pos'] = [tf.location.x, -tf.location.y, tf.location.z]
                    self.memory_vehicles[seq_id]['quat'] = self.carla_rot_to_ros_quat(tf.rotation)

    # ==========================================
    # 3. 辅助数学: 计算 Yaw 和 线框
    # ==========================================
    def get_yaw_degrees(self, quat_list):
        """ 从四元数计算 Yaw (-180 ~ 180) """
        (r, p, y) = euler_from_quaternion(quat_list)
        degrees = math.degrees(y)
        # 归一化其实 euler_from_quaternion 输出已经是 -pi 到 pi，转为度数即 -180 到 180
        return degrees

    def generate_wireframe_points(self, pos, quat, dims):
        """ 生成线框点用于 RViz """
        dx, dy, dz = dims[0]/2, dims[1]/2, dims[2]/2
        corners = np.array([
            [dx, dy, dz], [dx, dy, -dz], [dx, -dy, dz], [dx, -dy, -dz],
            [-dx, dy, dz], [-dx, dy, -dz], [-dx, -dy, dz], [-dx, -dy, -dz]
        ])
        rot_mat = quaternion_matrix(quat)[:3, :3]
        corners_global = np.dot(corners, rot_mat.T) + np.array(pos)
        
        lines_idx = [(0,1), (0,2), (0,4), (1,3), (1,5), (2,3), (2,6), (3,7), (4,5), (4,6), (5,7), (6,7)]
        points = []
        for p1_i, p2_i in lines_idx:
            p1, p2 = corners_global[p1_i], corners_global[p2_i]
            points.append(Point(*p1))
            points.append(Point(*p2))
        return points

    # ==========================================
    # 4. 发布核心
    # ==========================================
    def publish_data(self):
        if not self.memory_vehicles: return
        
        # --- 准备 RViz 数据 (MarkerArray) ---
        marker_msg = MarkerArray()
        
        # --- 准备 LLM 数据 (GlobalSceneDescription) ---
        scene_msg = GlobalVehicleDescription()
        scene_msg.header.stamp = rospy.Time.now()
        scene_msg.header.frame_id = "map"
        
        vehicle_list = []

        for seq_id, data in self.memory_vehicles.items():
            pos, quat, dims, rgba, c_text = data['pos'], data['quat'], data['dims'], data['rgba'], data['color_text']
            
            # ---------------------------------
            # A. 构建 LLM Msg
            # ---------------------------------
            v_desc = VehicleDescription()
            v_desc.id = seq_id
            v_desc.color = c_text
            v_desc.position = Point(*pos)
            v_desc.yaw = self.get_yaw_degrees(quat)
            v_desc.dimensions = Vector3(*dims)
            
            vehicle_list.append(v_desc)

            # ---------------------------------
            # B. 构建 RViz Marker (线框 + 文字)
            # ---------------------------------
            # 线框
            wire = Marker()
            wire.header = scene_msg.header
            wire.ns = "wireframes"
            wire.id = seq_id
            wire.type = Marker.LINE_LIST
            wire.action = Marker.ADD
            wire.pose.orientation.w = 1.0
            wire.scale.x = 0.05
            wire.color.r, wire.color.g, wire.color.b, wire.color.a = rgba
            wire.points = self.generate_wireframe_points(pos, quat, dims)
            wire.lifetime = rospy.Duration(0.3)
            marker_msg.markers.append(wire)
            
            # 文字 ID
            text = Marker()
            text.header = scene_msg.header
            text.ns = "ids"
            text.id = seq_id
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x, text.pose.position.y = pos[0], pos[1]
            text.pose.position.z = pos[2] + dims[2]/2 + 0.5
            text.scale.z = 0.8
            text.color.r, text.color.g, text.color.b, text.color.a = (1.0, 1.0, 1.0, 1.0)
            text.text = str(seq_id)
            text.lifetime = rospy.Duration(0.3)
            marker_msg.markers.append(text)

            # 文字 颜色 (可开关控制)
            if self.show_color_text:
                color_text = Marker()
                color_text.header = scene_msg.header
                color_text.ns = "colors"
                color_text.id = seq_id
                color_text.type = Marker.TEXT_VIEW_FACING
                color_text.action = Marker.ADD
                color_text.pose.position.x, color_text.pose.position.y = pos[0]-0.8, pos[1]
                color_text.pose.position.z = pos[2] + dims[2]/2 - 0.2  # 在ID下方
                color_text.scale.z = 0.6  # 颜色文字稍小
                color_text.color.r, color_text.color.g, color_text.color.b, color_text.color.a = (0.9, 0.9, 0.9, 1.0)  # 浅灰色
                color_text.text = c_text
                color_text.lifetime = rospy.Duration(0.3)
                marker_msg.markers.append(color_text)

            # 朝向箭头
            arrow = Marker()
            arrow.header = scene_msg.header
            arrow.ns = "directions"
            arrow.id = seq_id
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.orientation.w = 1.0
            arrow.scale.x = 0.4  # 箭身宽度
            arrow.scale.y = 1.0   # 箭头脑袋高度（变大）
            arrow.scale.z = 0.4  # 箭身深度
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = (1.0, 0.0, 0.0, 1.0)  # 红色箭头
            arrow.lifetime = rospy.Duration(0.3)

            # 箭头起点：车辆中心
            start_point = Point(pos[0], pos[1], pos[2])

            # 箭头终点：固定长度1.5米，根据yaw角度计算前方位置
            yaw_rad = math.radians(self.get_yaw_degrees(quat))
            arrow_length = 1.5  # 固定长度，较短
            end_x = pos[0] + arrow_length * math.cos(yaw_rad)
            end_y = pos[1] + arrow_length * math.sin(yaw_rad)
            end_z = pos[2]
            end_point = Point(end_x, end_y, end_z)

            arrow.points = [start_point, end_point]
            marker_msg.markers.append(arrow)

        # 填充总表
        scene_msg.total_count = len(vehicle_list)
        scene_msg.vehicles = vehicle_list

        # 发布
        self.pub_llm.publish(scene_msg)
        self.pub_viz.publish(marker_msg)

    # ==========================================
    # 5. 重置逻辑
    # ==========================================
    def reset_callback(self, msg):
        if msg.data:
            self.reset_requested = True

    def perform_reset(self):
        rospy.logwarn("Resetting vehicle map state...")
        
        # 1. 清除内部状态
        self.id_map.clear()
        self.memory_vehicles.clear()
        self.next_uid = 1
        
        # 2. 重新加载静态车辆（如果需要，否则它们会在下一帧重新被扫描进来，但为了ID一致性，这里只清空追踪ID即可）
        # 注意：register_vehicle 逻辑依赖 self.id_map 来判断是否是新车，清空后会重新注册生成新ID，符合预期。

        # 3. 清除 RViz 可视化
        marker_msg = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        delete_marker.header.frame_id = "map"
        marker_msg.markers.append(delete_marker)
        self.pub_viz.publish(marker_msg)

        # 4. 发布空的 LLM 消息
        empty_scene = GlobalVehicleDescription()
        empty_scene.header.stamp = rospy.Time.now()
        empty_scene.header.frame_id = "map"
        empty_scene.total_count = 0
        self.pub_llm.publish(empty_scene)

    def toggle_color_text_callback(self, req):
        """切换颜色文字显示的回调函数"""
        self.show_color_text = req.data
        status = "开启" if self.show_color_text else "关闭"
        rospy.loginfo(f"颜色文字显示已{status}")
        return {'success': True, 'message': f'颜色文字显示已{status}'}

    def run(self):
        # 频率设置为 5Hz (满足泊车场景需求，且节省算力)
        rate = rospy.Rate(5) 
        while not rospy.is_shutdown():
            try:
                # 检查重置请求
                if self.reset_requested:
                    self.perform_reset()
                    self.reset_requested = False

                self.update_perception()
                self.publish_data()
            except Exception as e:
                rospy.logwarn(f"Loop error: {e}")
            rate.sleep()

if __name__ == '__main__':
    try:
        PersistentVehicleMapper().run()
    except rospy.ROSInterruptException:
        pass
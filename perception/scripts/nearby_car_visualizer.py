#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import carla
import numpy as np
import math
from jsk_recognition_msgs.msg import BoundingBox, BoundingBoxArray
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

class NearbyCarlaViz:
    def __init__(self):
        rospy.init_node('nearby_car_visualizer', anonymous=True)
        
        # --- 参数配置 ---
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle')
        self.host = rospy.get_param('~host', 'localhost')
        self.port = rospy.get_param('~port', 2000)
        self.detection_radius = rospy.get_param('~radius', 8.0) # 搜索半径
        
        # 性能相关: 更新频率
        self.update_rate = 10 # Hz
        
        # --- 数据缓存容器 ---
        # 存储所有静态车的预计算 ROS 消息
        self.static_bboxes_cache = [] 
        # 存储所有静态车的 (x, y, z) 坐标，用于 numpy 加速计算
        self.static_locs_numpy = None 
        
        # --- 连接 Carla ---
        try:
            self.client = carla.Client(self.host, self.port)
            self.client.set_timeout(5.0)
            self.world = self.client.get_world()
            rospy.loginfo("Carla 连接成功")
        except Exception as e:
            rospy.logerr(f"Carla 连接失败: {e}")
            return

        # --- 解决 Label 问题 ---
        self.vehicle_label_type = self.resolve_vehicle_label()

        # --- 关键步骤：预加载静态资源 ---
        self.preprocess_static_objects()

        # --- ROS Publisher ---
        self.pub_jsk = rospy.Publisher('/carla_viz/all_vehicles', BoundingBoxArray, queue_size=10)
        
        rospy.loginfo("初始化完成，开始主循环...")

    def resolve_vehicle_label(self):
        """ 兼容不同 Carla 版本的 CityObjectLabel """
        for name in ['Vehicles', 'Car', 'Vehicle']:
            if hasattr(carla.CityObjectLabel, name):
                return getattr(carla.CityObjectLabel, name)
        return 10 # Default fallback

    def carla_rot_to_ros_quat(self, rot):
        """ 预计算用的旋转转换 """
        roll = math.radians(rot.roll)
        pitch = math.radians(-rot.pitch)
        yaw = math.radians(-rot.yaw)
        q = quaternion_from_euler(roll, pitch, yaw)
        return Quaternion(*q)

    def preprocess_static_objects(self):
        """
        核心优化函数：
        一次性获取所有静态车辆，将其转换为 ROS 消息并缓存。
        同时提取位置构建 numpy 数组，供运行时快速筛选。
        """
        rospy.loginfo("正在预处理静态环境车辆 (这可能需要几秒钟)...")
        
        # 1. 获取所有静态车辆对象
        env_objs = self.world.get_environment_objects(self.vehicle_label_type)
        
        msg_cache = []
        loc_cache = []
        
        # 2. 遍历并预生成消息
        for obj in env_objs:
            tf = obj.transform
            bb = obj.bounding_box
            
            # --- 构建 BoundingBox 消息 ---
            bbox = BoundingBox()
            bbox.header.frame_id = "map" # 静态物体始终相对于 map
            
            # 坐标转换 (Left-Hand -> Right-Hand)
            bbox.pose.position.x = tf.location.x
            bbox.pose.position.y = -tf.location.y
            bbox.pose.position.z = tf.location.z
            bbox.pose.orientation = self.carla_rot_to_ros_quat(tf.rotation)
            
            # 尺寸
            bbox.dimensions.x = bb.extent.x * 2
            bbox.dimensions.y = bb.extent.y * 2
            bbox.dimensions.z = bb.extent.z * 2
            
            # 样式 (静态车设为灰色/冷色)
            bbox.label = int(obj.id) % 2147483647
            bbox.value = 0.0 
            
            msg_cache.append(bbox)
            
            # 记录原始 Carla 坐标用于距离计算
            loc_cache.append([tf.location.x, tf.location.y, tf.location.z])
            
        self.static_bboxes_cache = msg_cache
        
        if len(loc_cache) > 0:
            self.static_locs_numpy = np.array(loc_cache)
        else:
            self.static_locs_numpy = np.empty((0, 3))
            
        rospy.loginfo(f"预处理完成: 缓存了 {len(self.static_bboxes_cache)} 辆静态车。")

    def run(self):
        rate = rospy.Rate(self.update_rate)
        
        while not rospy.is_shutdown():
            try:
                # 1. 获取自车 (每一帧都要找，因为可能会重置)
                actors = self.world.get_actors().filter('vehicle.*')
                ego_actor = None
                for actor in actors:
                    if actor.attributes.get('role_name') == self.role_name:
                        ego_actor = actor
                        break
                
                if ego_actor is None:
                    rate.sleep()
                    continue
                
                # 获取自车位置 (Carla 坐标)
                ego_loc = ego_actor.get_transform().location
                ego_pos_arr = np.array([ego_loc.x, ego_loc.y, ego_loc.z])
                
                # 准备输出的消息
                final_array = BoundingBoxArray()
                final_array.header.stamp = rospy.Time.now()
                final_array.header.frame_id = "map"
                
                # =========================================
                #  第一部分: 快速筛选静态车辆 (Numpy 加速)
                # =========================================
                if self.static_locs_numpy.shape[0] > 0:
                    # 向量化计算距离平方 (避免开根号，速度更快)
                    # dist_sq = (x-x0)^2 + (y-y0)^2 + (z-z0)^2
                    diff = self.static_locs_numpy - ego_pos_arr
                    dist_sq = np.sum(diff**2, axis=1)
                    
                    # 筛选出半径内的索引
                    radius_sq = self.detection_radius ** 2
                    nearby_indices = np.where(dist_sq < radius_sq)[0]
                    
                    # 直接从缓存中取出对应的消息加入列表
                    # 这里的循环次数 = 视野内的车辆数 (比如 5 辆)，而不是地图总车辆数 (比如 5000 辆)
                    for idx in nearby_indices:
                        final_array.boxes.append(self.static_bboxes_cache[idx])

                # =========================================
                #  第二部分: 处理动态车辆 (数量少，直接处理)
                # =========================================
                # actors 列表已经在上面获取过了
                for actor in actors:
                    if actor.id == ego_actor.id:
                        continue
                        
                    # 简单的距离判断
                    loc = actor.get_transform().location
                    dist = math.sqrt((loc.x - ego_loc.x)**2 + (loc.y - ego_loc.y)**2 + (loc.z - ego_loc.z)**2)
                    
                    if dist < self.detection_radius:
                        # 动态生成 ROS 消息
                        bbox = BoundingBox()
                        bbox.header = final_array.header
                        tf = actor.get_transform()
                        bb = actor.bounding_box
                        
                        bbox.pose.position.x = tf.location.x
                        bbox.pose.position.y = -tf.location.y
                        bbox.pose.position.z = tf.location.z
                        bbox.pose.orientation = self.carla_rot_to_ros_quat(tf.rotation)
                        
                        bbox.dimensions.x = bb.extent.x * 2
                        bbox.dimensions.y = bb.extent.y * 2
                        bbox.dimensions.z = bb.extent.z * 2
                        
                        bbox.label = actor.id
                        bbox.value = 1.0 # 动态车颜色不同 (红色/暖色)
                        
                        final_array.boxes.append(bbox)

                # 发布
                self.pub_jsk.publish(final_array)
                
            except Exception as e:
                rospy.logwarn(f"Loop Error: {e}")
            
            rate.sleep()

if __name__ == '__main__':
    try:
        node = NearbyCarlaViz()
        if node.client:
            node.run()
    except rospy.ROSInterruptException:
        pass
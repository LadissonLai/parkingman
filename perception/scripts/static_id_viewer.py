#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import carla
import math
import sys
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_euler

class StaticIDViewer:
    def __init__(self):
        rospy.init_node('static_id_viewer', anonymous=True)
        
        # --- 参数配置 ---
        self.role_name = rospy.get_param('~role_name', 'ego_vehicle') 
        self.scan_radius = rospy.get_param('~radius', 50.0) 
        
        # --- 连接 Carla ---
        self.host = rospy.get_param('~host', 'localhost')
        self.port = rospy.get_param('~port', 2000)
        
        try:
            client = carla.Client(self.host, self.port)
            client.set_timeout(5.0)
            self.world = client.get_world()
            rospy.loginfo("Carla 连接成功")
        except Exception as e:
            rospy.logerr(f"Carla 连接失败: {e}")
            sys.exit(1)

        self.pub_viz = rospy.Publisher('/tools/static_ids', MarkerArray, queue_size=1, latch=True)
        
        self.label_type = self.resolve_label()
        
        # --- 主流程 ---
        ego_loc = self.wait_for_ego_vehicle()
        if ego_loc:
            self.scan_and_visualize(ego_loc)
        
        rospy.spin()

    def resolve_label(self):
        for name in ['Vehicles', 'Car', 'Vehicle']:
            if hasattr(carla.CityObjectLabel, name):
                return getattr(carla.CityObjectLabel, name)
        return 10

    def wait_for_ego_vehicle(self):
        rospy.loginfo(f"正在寻找 role_name='{self.role_name}' 的车辆作为扫描中心...")
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            actors = self.world.get_actors().filter('vehicle.*')
            for actor in actors:
                if actor.attributes.get('role_name') == self.role_name:
                    loc = actor.get_transform().location
                    rospy.loginfo(f"已找到自车！坐标: ({loc.x:.1f}, {loc.y:.1f}, {loc.z:.1f})")
                    return loc
            rospy.loginfo_throttle(2, f"未找到自车 '{self.role_name}'...")
            rate.sleep()
        return None

    def carla_to_ros(self, tf):
        pos = Point(tf.location.x, -tf.location.y, tf.location.z)
        roll = math.radians(tf.rotation.roll)
        pitch = math.radians(-tf.rotation.pitch)
        yaw = math.radians(-tf.rotation.yaw)
        q = quaternion_from_euler(roll, pitch, yaw)
        return pos, Quaternion(*q)

    def scan_and_visualize(self, center_loc):
        rospy.loginfo(f"开始扫描以自车为中心 {self.scan_radius} 米范围内的静态车辆...")

        env_objs = self.world.get_environment_objects(self.label_type)
        marker_array = MarkerArray()
        found_data = [] # 存储真实 ID
        
        # 【关键修改】使用一个简单的计数器作为 ROS Marker 的内部 ID
        # 避免 CARLA 的巨大 ID 撑爆 ROS 的 int32
        ros_marker_id_counter = 0 
        
        for obj in env_objs:
            tf = obj.transform
            dist = math.sqrt((tf.location.x - center_loc.x)**2 + (tf.location.y - center_loc.y)**2)
            
            if dist > self.scan_radius:
                continue
            
            found_data.append(obj.id)
            pos, quat = self.carla_to_ros(tf)
            
            # --- Marker 1: 车身占位框 ---
            box = Marker()
            box.header.frame_id = "map"
            box.ns = "static_body"
            # 这里的 ID 只是为了让 ROS 区分不同的框，用计数器即可
            box.id = ros_marker_id_counter 
            ros_marker_id_counter += 1
            
            box.type = Marker.CUBE
            box.action = Marker.ADD
            box.pose.position = pos
            box.pose.orientation = quat
            box.scale.x = obj.bounding_box.extent.x * 2
            box.scale.y = obj.bounding_box.extent.y * 2
            box.scale.z = obj.bounding_box.extent.z * 2
            box.color.r = 1.0; box.color.g = 1.0; box.color.b = 1.0
            box.color.a = 0.2
            box.lifetime = rospy.Duration(0)
            marker_array.markers.append(box)

            # --- Marker 2: 巨大的 ID 文字 ---
            text = Marker()
            text.header.frame_id = "map"
            text.ns = "static_id"
            # 同样的，Marker ID 用计数器
            text.id = ros_marker_id_counter
            ros_marker_id_counter += 1
            
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos.x
            text.pose.position.y = pos.y
            text.pose.position.z = pos.z + 1.2
            text.scale.z = 1.5
            text.color.r = 1.0; text.color.g = 1.0; text.color.b = 0.0; text.color.a = 1.0
            
            # 【重点】这里才是你在 RViz 里看到的字
            # 我们把真实的巨大的 CARLA ID 放在这里，String 类型不会溢出
            text.text = str(obj.id) 
            
            text.lifetime = rospy.Duration(0)
            marker_array.markers.append(text)

        self.pub_viz.publish(marker_array)
        rospy.loginfo(f"扫描完成！找到 {len(found_data)} 辆静态车。")
        rospy.loginfo("请在 RViz 中查看 Topic: /tools/static_ids")
        
        # --- 打印字典模板 (使用真实 ID) ---
        print("\n" + "#"*60)
        print("【请复制下方代码，并对照 RViz 和 Carla 界面填色】")
        print("#"*60)
        print("self.STATIC_COLOR_DB = {")
        for uid in sorted(found_data):
            # 这里打印的是真实的 CARLA ID，用于你的代码逻辑
            print(f"    {uid}: (1.0, 1.0, 1.0, 1.0), # TODO: 修改颜色")
        print("}")
        print("#"*60 + "\n")

if __name__ == '__main__':
    try:
        StaticIDViewer()
    except rospy.ROSInterruptException:
        pass
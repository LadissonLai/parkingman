#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf.transformations as tft
from geometry_msgs.msg import PoseStamped, Quaternion, Point, Vector3, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import tf2_geometry_msgs
import numpy as np

class PlannerManager:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node('planner_manager', anonymous=True)

        # 初始化地图和 TF 数据
        self.map_frame_id = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.robot_frame_id = rospy.get_param('~robot_frame_id', 'ego_vehicle')

        # 初始化发布者
        self.goal_marker_pub = rospy.Publisher('/parking_goal_marker', Marker, queue_size=1)
        self.start_repub = rospy.Publisher('/parkman/planning/input/start', PoseWithCovarianceStamped, queue_size=1)
        self.goal_repub = rospy.Publisher('/parkman/planning/input/goal', PoseStamped, queue_size=1)
        
        # 初始化订阅者
        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        rospy.loginfo("Planner manager node initialized with TF-based start pose and coordinate transformation.")

    def map_callback(self, msg):
        """处理接收到的地图数据"""
        rospy.loginfo("Map received.")
        self.map_frame_id = msg.header.frame_id
        rospy.loginfo("Map frame ID: %s", self.map_frame_id)

    def goal_callback(self, goal_msg):
        """处理接收到的目标点话题，并通过 TF 获取起点"""
        rospy.loginfo("Received a new goal.")
        if self.map_frame_id is None:
            rospy.logwarn("Map not available yet.")
            return

        # 发布目标点 Marker
        self.publish_goal_marker(goal_msg)

        # 坐标系变换：确保目标点在地图坐标系下
        goal_pose = None
        if goal_msg.header.frame_id != self.map_frame_id:
            try:
                # 等待TF变换可用，最多等待1秒
                transform = self.tf_buffer.lookup_transform(
                    self.map_frame_id,
                    goal_msg.header.frame_id,
                    goal_msg.header.stamp,
                    rospy.Duration(1.0)
                )
                # 执行位姿变换
                goal_pose = tf2_geometry_msgs.do_transform_pose(goal_msg, transform)
                goal_pose.header.frame_id = self.map_frame_id
                goal_pose.header.stamp = rospy.Time.now()  # 更新时间戳

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr("TF2变换错误: %s" % str(e))
                return None
        else:
            goal_pose = goal_msg
        
        
        start_stamped = None
        try:
            # 查询从map到vehicle_frame的变换
            transform = self.tf_buffer.lookup_transform(
                self.map_frame_id,
                self.robot_frame_id,
                rospy.Time(0),  # 使用最新可用变换
                rospy.Duration(1.0)  # 等待最多1秒
            )

            # 构造PoseWithCovarianceStamped消息
            start_stamped = PoseWithCovarianceStamped()
            start_stamped.header.stamp = rospy.Time.now()
            start_stamped.header.frame_id = self.map_frame_id
            start_stamped.pose.pose.position = transform.transform.translation
            start_stamped.pose.pose.orientation = transform.transform.rotation
            covariance = np.diag([0.1, 0.1, 0.1, 0.01, 0.01, 0.01]).flatten().tolist()
            start_stamped.pose.covariance = covariance

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF查询失败: %s" % str(e))
            
        if start_stamped is not None and goal_pose is not None:
            # 发布起点
            self.start_repub.publish(start_stamped)
            # 发布目标点
            self.goal_repub.publish(goal_pose)

            rospy.loginfo("Start and goal positions published to /parkman/planning/input/start and /parkman/planning/input/goal.")

    def publish_goal_marker(self, goal_stamped):
        """发布目标点可视化 Marker"""
        marker = Marker()
        marker.header = goal_stamped.header
        marker.ns = "parking_goal"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = goal_stamped.pose
        marker.scale = Vector3(1.5, 0.3, 0.3)  # 箭头尺寸
        marker.color = ColorRGBA(0.0, 0.5, 1.0, 0.8)  # 蓝色
        marker.lifetime = rospy.Duration(0)  # 永久显示
        self.goal_marker_pub.publish(marker)
        rospy.loginfo("Published goal marker.")

if __name__ == '__main__':
    try:
        node = PlannerManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Planner manager node terminated.")
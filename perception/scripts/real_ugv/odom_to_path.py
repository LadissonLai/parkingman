#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class OdomToPath:
    def __init__(self):
        self.path_pub = rospy.Publisher('/path', Path, queue_size=10)
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_callback, queue_size=100)

        self.path_msg = Path()
        self.path_msg.header.frame_id = "camera_init"   # 建议和里程计坐标系一致

    def odom_callback(self, msg):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        # Path 的 header 一般与路径所在坐标系一致
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = msg.header.frame_id

        self.path_msg.poses.append(pose)

        self.path_pub.publish(self.path_msg)

if __name__ == '__main__':
    rospy.init_node('odom_to_path')
    OdomToPath()
    rospy.spin()
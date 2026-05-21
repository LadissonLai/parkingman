#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
One-shot ROS node: read gt_trajectory[0] from task_dir, publish set_transform
to place the ego vehicle at the task starting pose.

Usage:
  rosrun carla_close_test init_ego.py _task_dir:=/path/to/task/folder
"""

import os
import sys
import math

import rospy
import pandas as pd
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


def main():
    rospy.init_node('init_ego', anonymous=False)

    task_dir = rospy.get_param('~task_dir', '')
    if not task_dir:
        rospy.logerr("~task_dir param not set. Exiting.")
        sys.exit(1)

    gt_csv = os.path.join(task_dir, 'gt_trajectory.csv')
    if not os.path.isfile(gt_csv):
        rospy.logerr("gt_trajectory.csv not found in: %s", task_dir)
        sys.exit(1)

    df = pd.read_csv(gt_csv)
    if df.empty:
        rospy.logerr("gt_trajectory.csv is empty: %s", gt_csv)
        sys.exit(1)

    first = df.iloc[0]
    x   = float(first['x'])
    y   = float(first['y'])
    z   = float(first['z'])
    yaw = float(first['yaw'])  # radians, CARLA map frame

    if z < 0.25:
        z = 0.25

    q = quaternion_from_euler(0.0, 0.0, yaw)

    pose = Pose()
    pose.position.x    = x
    pose.position.y    = y
    pose.position.z    = z
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]

    pub = rospy.Publisher(
        '/carla/ego_vehicle/control/set_transform',
        Pose,
        queue_size=10
    )

    rospy.loginfo("Placing ego at start: x=%.3f y=%.3f z=%.3f yaw=%.3f rad", x, y, z, yaw)

    rate = rospy.Rate(1.0)
    for _ in range(5):
        if rospy.is_shutdown():
            break
        pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Ego initialized at task start pose.")


if __name__ == '__main__':
    main()

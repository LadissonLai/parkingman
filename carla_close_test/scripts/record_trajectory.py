#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Description:
This script is used to record the trajectory of the ego vehicle in CARLA.
It runs as a ROS node and subscribes to the CARLA ego vehicle's odometry topic (e.g., `/carla/ego_vehicle/odometry`).
You can start and stop recording by pressing [Enter] in the terminal.

Save Format:
After recording is stopped, the trajectory data is saved chronologically into a CSV file named with the current timestamp.
The recorded data columns are:
- timestamp : Timestamp of the message (in seconds)
- x, y, z   : 3D position of the ego vehicle in the global coordinate system
- roll, pitch, yaw : Ego vehicle attitude Euler angles converted from quaternion (in radians)
"""

import os
import rospy
import csv
import tf
from nav_msgs.msg import Odometry
from datetime import datetime
import sys

# Support input for both Python 2 and Python 3
try:
    input_func = raw_input
except NameError:
    input_func = input

class TrajectoryRecorder:
    def __init__(self):
        rospy.init_node('trajectory_recorder_node', anonymous=True)
        self.recording = False
        self.trajectory_data = []

        # Get town name from parameter server or default to town04
        self.town_name = rospy.get_param('~town_name', 'town04')
        
        # Subscribe to CARLA ego vehicle odometry topic
        self.odom_sub = rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.odom_callback)
        
    def odom_callback(self, msg):
        if not self.recording:
            return
            
        # Extract timestamp
        timestamp = msg.header.stamp.to_sec()
        
        # Extract XYZ coordinates
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Extract orientation and convert to Euler angles (roll, pitch, yaw)
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([qx, qy, qz, qw])
        
        # Save the current frame data
        self.trajectory_data.append([timestamp, x, y, z, roll, pitch, yaw])

    def save_to_csv(self):
        if len(self.trajectory_data) == 0:
            rospy.logwarn("No trajectory data recorded!")
            return

        # Determine the base directory (test_datasets inside the package)
        script_dir = os.path.dirname(os.path.abspath(__file__))
        pkg_dir = os.path.dirname(script_dir)
        base_save_dir = os.path.join(pkg_dir, "test_datasets", self.town_name)
        
        # Create timestamped sub-directory
        dir_name = datetime.now().strftime('%Y-%m-%d-%H-%M')
        save_dir = os.path.join(base_save_dir, dir_name)
        
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        filename = os.path.join(save_dir, "gt_trajectory.csv")
        
        # Cross-version compatibility for file writing
        mode = 'w' if sys.version_info[0] == 3 else 'wb'
        newline_kwargs = {'newline': ''} if sys.version_info[0] == 3 else {}

        with open(filename, mode, **newline_kwargs) as file:
            writer = csv.writer(file)
            writer.writerow(['timestamp', 'x', 'y', 'z', 'roll', 'pitch', 'yaw'])
            writer.writerows(self.trajectory_data)
            
        rospy.loginfo("Successfully saved {} trajectory points to file: {}".format(len(self.trajectory_data), filename))

    def run(self):
        print("="*50)
        print("Ready to record trajectory...")
        input_func("Press [Enter] to start recording: ")
        
        self.recording = True
        rospy.loginfo("Started recording trajectory points...")
        
        input_func("Press [Enter] again to stop recording and save the file: ")
        
        self.recording = False
        rospy.loginfo("Recording stopped, preparing to save...")
        
        # Unregister subscriber and save file
        self.odom_sub.unregister()
        self.save_to_csv()
        rospy.signal_shutdown("Recording completed, shutting down.")

if __name__ == '__main__':
    try:
        recorder = TrajectoryRecorder()
        recorder.run()
    except rospy.ROSInterruptException:
        pass

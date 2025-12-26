#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
from std_msgs.msg import Bool
from carla_msgs.msg import CarlaEgoVehicleControl

class TrajectoryTracker:
    def __init__(self):
        """
        Initialize the trajectory tracker
        """
        # Initialize ROS node
        rospy.init_node('trajectory_tracker', anonymous=True)

        # Get parameters
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.tracking_rate = rospy.get_param('~tracking_rate', 20.0) # 0.1 sec per step

        # --- State Management ---
        self.current_path = None
        self.current_path_index = 0
        self.tracking_active = False # Flag to indicate if we are supposed to be tracking a path
        self.is_paused = False

        # Create TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher for control commands
        self.control_pub = rospy.Publisher(
            '/carla/ego_vehicle/control/set_transform',
            Pose,
            queue_size=10
        )
        
        self.stop_vehicle_pub = rospy.Publisher(
            '/carla/ego_vehicle/vehicle_control_cmd',
            CarlaEgoVehicleControl,
            queue_size=1
        )

        # Publisher for goal reached status
        self.status_pub = rospy.Publisher(
            '/trajectory/goal_reached',
            Bool, 
            queue_size=1
        )

        # --- Subscribers ---
        # Subscriber for the planned trajectory
        self.path_sub = rospy.Subscriber(
            '/controller/input/trajectory',
            Path,
            self.path_callback
        )
        
        # Subscriber to enable/disable (pause/resume) tracking
        self.enable_sub = rospy.Subscriber(
            '/trajectory_tracker/enable',
            Bool,
            self.enable_callback
        )
        
        # Core Logic: Non-blocking Timer
        self.tracking_timer = rospy.Timer(rospy.Duration(1.0 / self.tracking_rate), self.tracking_loop)

        rospy.on_shutdown(self.shutdown_hook)

        rospy.loginfo("Trajectory tracker initialized with Pause/Resume capability.")
        rospy.loginfo("Publish to /trajectory_tracker/enable (Bool): False=Pause, True=Resume.")

    def enable_callback(self, msg):
        """
        Callback to handle pause/resume commands.
        """
        if msg.data:
            if self.is_paused:
                rospy.loginfo("Trajectory tracking RESUMED.")
            self.is_paused = False
            self.stop_vehicle_pub.publish(CarlaEgoVehicleControl(brake=0.0))
        else:
            if not self.is_paused:
                rospy.loginfo("Trajectory tracking PAUSED.")
            self.is_paused = True
            for _ in range(3):
                self.stop_vehicle_pub.publish(CarlaEgoVehicleControl(brake=1.0))

    def path_callback(self, msg):
        """
        Callback for new trajectory messages. This resets the state
        and lets the tracking_loop handle the new path.
        """
        if not msg.poses:
            rospy.logwarn("Received an empty path. Stopping tracking.")
            self.tracking_active = False
            self.current_path = None
            return

        rospy.loginfo(f"Received new path with {len(msg.poses)} points. Restarting tracking.")
        
        self.current_path = msg
        self.current_path_index = 0
        self.tracking_active = True
        self.is_paused = False
        
        self.status_pub.publish(Bool(data=False))


    def tracking_loop(self, event=None):
        """
        This function is called periodically. It publishes one point at a time.
        It will now halt if the 'is_paused' flag is True.
        """
        # If tracking is not active, or is paused, or path is invalid, do nothing.
        if not self.tracking_active or self.is_paused or not self.current_path or not self.current_path.poses:
            return

        # Check if we have finished the trajectory
        if self.current_path_index >= len(self.current_path.poses):
            rospy.loginfo("Trajectory following completed.")
            self.tracking_active = False
            self.current_path = None 
            self.status_pub.publish(Bool(data=True))
            for _ in range(3):
                self.stop_vehicle_pub.publish(CarlaEgoVehicleControl(brake=1.0))
            #self.stop_vehicle_pub.publish(CarlaEgoVehicleControl(brake=0.0))
            return

        pose_stamped = self.current_path.poses[self.current_path_index]

        if pose_stamped.header.frame_id != self.map_frame:
            transformed_pose = self.transform_pose(pose_stamped, self.map_frame)
            if transformed_pose is None:
                rospy.logwarn(f"Failed to transform path point {self.current_path_index}, skipping.")
                self.current_path_index += 1
                return
        else:
            transformed_pose = pose_stamped
            
        control_pose = Pose()
        control_pose.position = transformed_pose.pose.position
        if control_pose.position.z <= 0.05:
            control_pose.position.z = 0.25
        control_pose.orientation = transformed_pose.pose.orientation
        self.control_pub.publish(control_pose)

        self.current_path_index += 1

    def transform_pose(self, input_pose, target_frame):
        """
        Transform a PoseStamped message to the target frame. (No changes)
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                input_pose.header.frame_id,
                rospy.Time(0), 
                rospy.Duration(1.0)
            )
            transformed_pose = tf2_geometry_msgs.do_transform_pose(input_pose, transform)
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"TF2 transform error: {e}")
            return None
    
    def shutdown_hook(self):
        """
        Function called on node shutdown. (No changes)
        """
        if self.tracking_active:
            self.status_pub.publish(Bool(data=False))
            rospy.logwarn("Node shutdown during trajectory tracking. Goal not reached.")
        rospy.loginfo("Trajectory tracker node terminated.")


if __name__ == '__main__':
    try:
        tracker = TrajectoryTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
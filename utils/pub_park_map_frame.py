#!/usr/bin/env python3

import rospy
import tf
import tf2_ros
import geometry_msgs.msg

def static_tf_broadcaster():
    """
    Publishes a static transform from a parent frame to a child frame.
    """
    rospy.init_node('static_tf_broadcaster', anonymous=True)

    # Create a TransformStamped message
    static_transform_stamped = geometry_msgs.msg.TransformStamped()

    # Set the header information
    static_transform_stamped.header.stamp = rospy.Time.now()
    static_transform_stamped.header.frame_id = "map"  # Parent frame
    static_transform_stamped.child_frame_id = "parking_start_map"  # Child frame

    # Set the translation (x, y, z) in meters
    static_transform_stamped.transform.translation.x = 290.89 + 55 - 8
    static_transform_stamped.transform.translation.y = 232.72 - 20 - 10
    static_transform_stamped.transform.translation.z = 0.05

    # Set the rotation as a quaternion (roll, pitch, yaw)
    # tf.transformations.quaternion_from_euler converts Euler angles to a quaternion
    # roll=0, pitch=0, yaw=90 degrees (1.5708 radians)
    q = tf.transformations.quaternion_from_euler(0, 0, -180)
    static_transform_stamped.transform.rotation.x = q[0]
    static_transform_stamped.transform.rotation.y = q[1]
    static_transform_stamped.transform.rotation.z = q[2]
    static_transform_stamped.transform.rotation.w = q[3]

    # Create a StaticTransformBroadcaster
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    # Publish the transform
    # The transform will be latched on the /tf_static topic.
    # It will only be sent once when the node starts, and any new subscribers will receive it.
    static_broadcaster.sendTransform(static_transform_stamped)
    
    rospy.loginfo("Static transform published! Parent: %s, Child: %s", 
                  static_transform_stamped.header.frame_id, 
                  static_transform_stamped.child_frame_id)
                  
    # Keep the node running so the transform is available for other nodes
    rospy.spin()

if __name__ == '__main__':
    try:
        static_tf_broadcaster()
    except rospy.ROSInterruptException:
        pass
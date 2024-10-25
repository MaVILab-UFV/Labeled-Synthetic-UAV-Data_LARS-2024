#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates
import tf
from geometry_msgs.msg import TransformStamped

class GazeboToTF:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('gazebo_to_tf', anonymous=True)
        
        # Create a TF broadcaster
        self.tf_broadcaster = tf.TransformBroadcaster()

        # Create a TF listener
        self.tf_listener = tf.TransformListener()
        
        # Store the previous pose information
        self.prev_pose = {}
        
        # Subscribe to the /gazebo/link_states topic
        self.link_states_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, self.link_states_callback)
        
    def link_states_callback(self, msg):
        for i, link_name in enumerate(msg.name):
            # Extract the pose
            position = msg.pose[i].position
            orientation = msg.pose[i].orientation
            
            # Check if the pose has changed
            if link_name in self.prev_pose and self.prev_pose[link_name] == (position, orientation):
                continue
        
            # Store the current pose
            self.prev_pose[link_name] = (position, orientation)
        
            # Get the current time
            current_time = rospy.Time.now()
        
            try:
                # Lookup the transform
                self.tf_listener.waitForTransform("world", link_name, current_time, rospy.Duration(0.1))
            except tf.Exception as e:
                # If the transform is not available, broadcast it
                self.tf_broadcaster.sendTransform(
                    (position.x, position.y, position.z),
                    (orientation.x, orientation.y, orientation.z, orientation.w),
                    current_time,
                    link_name,
                    "world"
                )

if __name__ == '__main__':
    try:
        GazeboToTF()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
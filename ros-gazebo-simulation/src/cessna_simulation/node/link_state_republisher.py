#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import LinkStates, LinkState

# Global variable to store the previous pose
prev_position = None

def link_states_callback(data):
    global prev_position

    try:
        # Get the index of the link "cessna_c172::cessna_c172::body"
        index = data.name.index('cessna_c172::cessna_c172::body')
        
        # Extract the current pose
        current_position = data.pose[index].position
        
        # Check if the position has changed
        if prev_position is not None and current_position == prev_position:
            return
        
        # Store the current position
        prev_position = current_position
        
        # Create the LinkState message
        link_state_msg = LinkState()
        link_state_msg.link_name = 'cessna_c172_sensors::base_link'
        
        # Adjust the pose
        link_state_msg.pose = data.pose[index]
        link_state_msg.pose.position.z -= 0.04
        link_state_msg.pose.position.x -= 0.1
        
        # Copy the twist
        link_state_msg.twist = data.twist[index]
        
        # Set the reference frame to 'world'
        link_state_msg.reference_frame = 'world'

        # Publish the LinkState message
        set_link_state_pub.publish(link_state_msg)
    except ValueError:
        rospy.logerr('Link cessna_c172::cessna_c172::body not found in link_states topic')

if __name__ == '__main__':
    # Set use_sim_time to true
    rospy.set_param('/use_sim_time', True)
    
    rospy.init_node('link_state_republisher')

    # Create a publisher for the /gazebo/set_link_state topic
    set_link_state_pub = rospy.Publisher('/gazebo/set_link_state', LinkState, queue_size=1)

    # Subscribe to the /gazebo/link_states topic
    rospy.Subscriber('/gazebo/link_states', LinkStates, link_states_callback)
    
    rospy.spin()

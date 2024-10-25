#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
import math
from tqdm import tqdm
import os

class WaypointPublisher:
    def __init__(self, waypoint_file):
        self.waypoints = self.read_waypoints(waypoint_file)
        self.current_waypoint_index = 0
        self.current_pose = None
        
        rospy.init_node('waypoint_publisher', anonymous=True)
        
        self.pub = rospy.Publisher('/command/pose', PoseStamped, queue_size=10)
        rospy.Subscriber('/ground_truth/state', Odometry, self.odometry_callback)
        
        self.rate = rospy.Rate(10)  # Publica a 10 Hz
        self.progress_bar = tqdm(total=len(self.waypoints), desc='Waypoints Progress')

    def read_waypoints(self, file_path):
        waypoints = np.loadtxt(file_path, delimiter=',', skiprows=1)
        print(waypoints.shape)
        waypoints[:,0] = waypoints[:,0]
        waypoints[:,1] = waypoints[:,1]
        return waypoints

    def odometry_callback(self, data):
        self.current_pose = data.pose.pose

    def distance_to_waypoint(self, waypoint):
        if self.current_pose is None:
            return float('inf')
        
        dx = self.current_pose.position.x - waypoint[0]
        dy = self.current_pose.position.y - waypoint[1]
        dz = self.current_pose.position.z - waypoint[2]
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def publish_waypoints(self):
        while not rospy.is_shutdown():
            if self.current_waypoint_index < len(self.waypoints):
                waypoint = self.waypoints[self.current_waypoint_index]
                
                distance = self.distance_to_waypoint(waypoint)
                if distance < 0.2:  # 20 cm de tolerância
                    rospy.loginfo(f"Reached waypoint: ({waypoint[0]}, {waypoint[1]}, {waypoint[2]})")
                    self.current_waypoint_index += 1
                    self.progress_bar.update(1)
                    if self.current_waypoint_index >= len(self.waypoints) :
                        rospy.loginfo("All waypoints reached.")
                        self.progress_bar.close()
                        self.shutdown_sequence()
                        break
                    waypoint = self.waypoints[self.current_waypoint_index]

                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = 'world'
                pose.pose.position.x = waypoint[0]
                pose.pose.position.y = waypoint[1]
                pose.pose.position.z = waypoint[2]
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0  # Sem rotação

                rospy.loginfo(f"Publishing waypoint: ({waypoint[0]}, {waypoint[1]}, {waypoint[2]})")
                self.pub.publish(pose)
            self.rate.sleep()

    def shutdown_sequence(self):

        rospy.loginfo("Shutting down Gazebo...")
        os.system("pkill gzserver")
        os.system("pkill gzclient")

        rospy.loginfo("Shutting down ROS...")
        os.system("pkill roscore")
        os.system("pkill rosmaster")

        rospy.loginfo("Shutting down the program...")
        os.kill(os.getpid(), signal.SIGINT)

if __name__ == '__main__':
    rospy.init_node('waypoint_publisher', anonymous=True)
    waypoint_file = rospy.get_param('~waypoint_file', 'zigzag_path.txt')
    
    try:
        waypoint_publisher = WaypointPublisher(waypoint_file)
        waypoint_publisher.publish_waypoints()
    except rospy.ROSInterruptException:
        pass
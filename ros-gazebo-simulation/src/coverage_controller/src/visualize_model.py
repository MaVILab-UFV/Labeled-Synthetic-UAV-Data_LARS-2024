#!/usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
import numpy as np

class ModelVisualizer:
    def __init__(self):
        rospy.init_node('model_visualizer', anonymous=True)
        
        self.model_name = "mesh"
        self.model_file = rospy.get_param('~model_file', 'model/mesh.dae')
        
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        self.rate = rospy.Rate(10)  # 10 Hz
        self.current_pose = None
        self.last_pose = None

    def model_states_callback(self, data):
        try:
            index = data.name.index(self.model_name)
            self.current_pose = data.pose[index]
            
            if self.last_pose is None or not self.pose_equal(self.current_pose, self.last_pose):
                self.publish_marker()
                self.last_pose = self.current_pose
        except ValueError:
            pass
            # rospy.logwarn(f"Model {self.model_name} not found in /gazebo/model_states")

    def pose_equal(self, pose1, pose2):
        return np.allclose([pose1.position.x, pose1.position.y, pose1.position.z,
                            pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w],
                           [pose2.position.x, pose2.position.y, pose2.position.z,
                            pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w],
                           atol=1e-6)

    def publish_marker(self):
        if self.current_pose:
            marker = Marker()
            marker.header.frame_id = "world"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "model"
            marker.id = 0
            marker.type = Marker.MESH_RESOURCE
            marker.mesh_resource = f"file://{self.model_file}"
            marker.mesh_use_embedded_materials = True
            marker.action = Marker.ADD
            marker.pose = self.current_pose
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.a = 1.0
            marker.color.r = 0.5
            marker.color.g = 0.5
            marker.color.b = 0.5

            self.marker_pub.publish(marker)

if __name__ == '__main__':
    try:
        visualizer = ModelVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

import rospy
import numpy as np

import tf
from tf.transformations import euler_from_quaternion

from visual_servoing.msg import ConeLocation
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

class SimMarker():
    """
    Rosnode for handling simulated cone. Listens for clicked point
    in rviz and publishes a marker. Publishes position of cone
    relative to robot for Parking Controller to park in front of.
    """
    def __init__(self):
        # Subscribe to clicked point messages from rviz    
        rospy.Subscriber("/clicked_point", 
            PointStamped, self.clicked_callback)
        self.message_x = None
        self.message_y = None
        self.message_frame = "map"

        self.cone_pub = rospy.Publisher("/relative_cone", 
            ConeLocation,queue_size=1)
        self.marker_pub = rospy.Publisher("/cone_marker",
            Marker, queue_size=1)
        self.tf_listener = tf.TransformListener()
        self.rate = rospy.Rate(10) #10 hz
        
        while not rospy.is_shutdown():
            self.publish_cone()
            self.rate.sleep()

    def publish_cone(self):
        """
        Publish the relative location of the cone
        """
        # Find out most recent relative location of cone
        if self.message_x is None:
            return
        try:
            msg_frame_pos, msg_frame_quat = self.tf_listener.lookupTransform(
                "base_link", self.message_frame, rospy.Time(0))
        except:
            return
        # Using relative transformations, convert cone in whatever frame rviz
        # was in to cone in base link (which is the control frame)
        (roll, pitch, yaw) = euler_from_quaternion(msg_frame_quat)
        cone_relative_baselink_x =\
            msg_frame_pos[0]+np.cos(yaw)*self.message_x-np.sin(yaw)*self.message_y 
        cone_relative_baselink_y =\
            msg_frame_pos[1]+np.cos(yaw)*self.message_y+np.sin(yaw)*self.message_x
        
        # Publish relative cone location
        relative_cone = ConeLocation()
        relative_cone.x_pos = cone_relative_baselink_x
        relative_cone.y_pos = cone_relative_baselink_y
        self.cone_pub.publish(relative_cone)

    def draw_marker(self):
        """
        Publish a marker to represent the cone in rviz
        """
        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.message_x
        marker.pose.position.y = self.message_y
        self.marker_pub.publish(marker)


    def clicked_callback(self, msg):
        # Store clicked point in the map frame
        msg_frame_pos, msg_frame_quat = self.tf_listener.lookupTransform(
               self.message_frame, msg.header.frame_id, rospy.Time(0))
        (roll, pitch, yaw) = euler_from_quaternion(msg_frame_quat)

        self.message_x = \
            msg_frame_pos[0]+np.cos(yaw)*msg.point.x\
            -np.sin(yaw)*msg.point.y
        
        self.message_y = \
            msg_frame_pos[1]+np.cos(yaw)*msg.point.y\
            +np.sin(yaw)*msg.point.x
        
        # Draw a marker for visualization
        self.draw_marker()

if __name__ == '__main__':
    try:
        rospy.init_node('SimMarker', anonymous=True)
        SimMarker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
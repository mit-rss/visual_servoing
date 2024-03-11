#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros
from tf_transformations import euler_from_quaternion

from vs_msgs.msg import ConeLocation
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped

class SimMarker(Node):
    """
    Rosnode for handling simulated cone. Listens for clicked point
    in rviz and publishes a marker. Publishes position of cone
    relative to robot for Parking Controller to park in front of.
    """
    def __init__(self):
        super().__init__("cone_sim_marker")
        # Subscribe to clicked point messages from rviz    
        self.create_subscription(PointStamped,
            "/clicked_point", self.clicked_callback, 1)

        self.message_x = None
        self.message_y = None
        self.message_frame = "map"

        self.cone_pub = self.create_publisher(ConeLocation, "/relative_cone", 1)
        self.marker_pub = self.create_publisher(Marker, "/cone_marker", 1)

        self.tfBuffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.cone_pub_timer = self.create_timer(0.1, self.publish_cone)

        self.get_logger().info("Cone Sim Marker Initialized")

    def publish_cone(self):
        """
        Publish the relative location of the cone
        """
        # Find out most recent relative location of cone
        if self.message_x is None:
            # print("no message")
            return
        try:
            t = self.tfBuffer.lookup_transform(
                "base_link", self.message_frame, rclpy.time.Time())

            msg_frame_pos = t.transform.translation
            msg_frame_quat = t.transform.rotation
            msg_frame_quat = [msg_frame_quat.x, msg_frame_quat.y,
                            msg_frame_quat.z, msg_frame_quat.w]
            msg_frame_pos = [msg_frame_pos.x, msg_frame_pos.y, msg_frame_pos.z]
        except:
            # print("no transform")
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
        t = self.tfBuffer.lookup_transform(
            self.message_frame, msg.header.frame_id, rclpy.time.Time())
        
        msg_frame_pos = t.transform.translation
        msg_frame_quat = t.transform.rotation
        msg_frame_quat = [msg_frame_quat.x, msg_frame_quat.y,
                          msg_frame_quat.z, msg_frame_quat.w]
        msg_frame_pos = [msg_frame_pos.x, msg_frame_pos.y, msg_frame_pos.z]
        
        (roll, pitch, yaw) = euler_from_quaternion(msg_frame_quat)

        self.message_x = \
            msg_frame_pos[0]+np.cos(yaw)*msg.point.x\
            -np.sin(yaw)*msg.point.y
        
        self.message_y = \
            msg_frame_pos[1]+np.cos(yaw)*msg.point.y\
            +np.sin(yaw)*msg.point.x
        
        # Draw a marker for visualization
        self.draw_marker()

def main(args=None):
    rclpy.init(args=args)
    sim_marker = SimMarker()
    rclpy.spin(sim_marker)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
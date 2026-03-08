#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

import tf2_ros
from scipy.spatial.transform import Rotation as R

from vs_msgs.msg import ConeLocation
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PointStamped


class SimMarker(Node):
    """
    ROS node for handling simulated cone. Listens for clicked point
    in rviz and publishes a marker. Publishes position of cone
    relative to robot for Parking Controller to park in front of.
    """

    def __init__(self):
        super().__init__("cone_sim_marker")

        self.create_subscription(
            PointStamped, "/clicked_point", self.clicked_callback, 1
        )

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
        """Publish the relative location of the cone"""

        if self.message_x is None:
            return

        try:
            t = self.tfBuffer.lookup_transform(
                "base_link", self.message_frame, rclpy.time.Time()
            )

            trans = t.transform.translation
            quat = t.transform.rotation

            msg_frame_quat = [quat.x, quat.y, quat.z, quat.w]
            msg_frame_pos = [trans.x, trans.y, trans.z]

        except Exception:
            return

        yaw = R.from_quat(msg_frame_quat).as_euler("xyz")[2]

        cone_relative_baselink_x = (
            msg_frame_pos[0]
            + np.cos(yaw) * self.message_x
            - np.sin(yaw) * self.message_y
        )

        cone_relative_baselink_y = (
            msg_frame_pos[1]
            + np.cos(yaw) * self.message_y
            + np.sin(yaw) * self.message_x
        )

        relative_cone = ConeLocation()
        relative_cone.x_pos = cone_relative_baselink_x
        relative_cone.y_pos = cone_relative_baselink_y

        self.cone_pub.publish(relative_cone)

    def draw_marker(self):
        """Publish a marker to represent the cone in rviz"""

        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5

        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.message_x
        marker.pose.position.y = self.message_y

        self.marker_pub.publish(marker)

    def clicked_callback(self, msg):

        t = self.tfBuffer.lookup_transform(
            self.message_frame, msg.header.frame_id, rclpy.time.Time()
        )

        trans = t.transform.translation
        quat = t.transform.rotation

        msg_frame_quat = [quat.x, quat.y, quat.z, quat.w]
        msg_frame_pos = [trans.x, trans.y, trans.z]

        yaw = R.from_quat(msg_frame_quat).as_euler("xyz")[2]

        self.message_x = (
            msg_frame_pos[0]
            + np.cos(yaw) * msg.point.x
            - np.sin(yaw) * msg.point.y
        )

        self.message_y = (
            msg_frame_pos[1]
            + np.cos(yaw) * msg.point.y
            + np.sin(yaw) * msg.point.x
        )

        self.draw_marker()


def main(args=None):
    rclpy.init(args=args)
    sim_marker = SimMarker()
    rclpy.spin(sim_marker)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

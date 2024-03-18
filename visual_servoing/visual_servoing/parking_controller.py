#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from visual_servoing.PID import *
from ackermann_msgs.msg import AckermannDriveStamped
import time

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")
        self.start_time = time.time()
        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = 1.0 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.L = 0.3 #meters, wheel base of the car
        self.v = 0.0 #velocity of the car
        self.delta = 0.0 #steering angle

        self.theta_controller = PID(self, 2, 0, 0)
        self.speed_controller = PID(self, 1, 0, 0)

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        drive_cmd.header.stamp = self.get_clock().now().to_msg()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd
        x = self.relative_x
        y = self.relative_y

        theta = np.tan(y/x)
        d = (x**2+y**2)**.5

        min_turning_radius = self.L/np.tan(0.34)
        min_turning_radius = min_turning_radius*1

        critical_radius = (min_turning_radius**2+self.parking_distance**2)**0.5

        if ((x**2 + (y-min_turning_radius)**2) < critical_radius**2) or (x**2 + (y+min_turning_radius**2) < critical_radius**2): #handles the case where the cone is in the dead area
            u_speed = -0.5

            # end_time = time.time()
            # dt = self.start_time-end_time
            u_steering = self.theta_controller(theta)
            #this section helps fix the annoying spots where it gets confused
            if y > 0:
                u_steering = -abs(u_steering)
            elif y < 0:
                u_steering = abs(u_steering)

        else:
            # thetacontroller = PIDController(-2,0,0,0)
            # end_time = time.time()
            # dt = self.start_time-end_time
            u_steering = -self.theta_controller(theta)

            #this section helps fix the annoying spots where it gets confused
            if y > 0:
                u_steering = abs(u_steering)
            elif y < 0:
                u_steering = -abs(u_steering)
            


            # speedcontroller = PIDController(-1,0,0,self.parking_distance)
            u_speed = self.speed_controller(d)

        #publish drive command
        u_speed = np.clip(u_speed,-2.,2.) #limit the speed setting
        drive_cmd.drive.speed = u_speed
        self.v = u_speed

        u_steering = np.clip(u_steering,-0.34,0.34) #limit the steering angle
        drive_cmd.drive.steering_angle = u_steering
        self.delta = u_steering

        self.get_logger().info(f"speed: {u_speed}  |  steering: {u_steering}")

        #################################

        if drive_cmd.drive.speed > 0.01:
            self.drive_pub.publish(drive_cmd)
        self.error_publisher()
        self.start_time = time.time()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)
        x = self.relative_x
        y = self.relative_y

        d = (x**2+y**2)**.5

        d_error = d - self.parking_distance
        x_error = x - self.parking_distance
        y_error = y

        error_msg.x_error = x_error
        error_msg.y_error = y_error
        error_msg.distance_error = d_error

        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
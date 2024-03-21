#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")

        self.declare_parameter("drive_topic")
        DRIVE_TOPIC = self.get_parameter("drive_topic").value # set in launch file; different for simulator vs racecar

        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 3)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        self.parking_distance = 0# .75 # meters; try playing with this number!
        self.relative_x = 0
        self.relative_y = 0

        self.need_to_back_up = False
        self.last_wheel_angle = 0.0
        self.last_car_velocity = 0.0
        self.desired_distance_delta = 0.1 # meters
        self.approach_angle_delta = 0.2 # rad

        self.in_happy = False

        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        #################################

        # YOUR CODE HERE
        # Use relative position and your control law to set drive_cmd

        #################################
        self.max_steering_angle = 0.34 # radians
        self.wheelbase = 0.3302 # Wheelbase, meters
        self.car_min_turning_radius = self.wheelbase / np.tan(self.max_steering_angle) # meters, ~ 0.9335

        angle_to_cone = np.arctan2(self.relative_y, self.relative_x) # radians
        distance_to_cone = np.sqrt(self.relative_x**2 + self.relative_y**2) # meters

        self.forward_speed = 0.75 #min(0.75, np.sqrt(abs(distance_to_cone-self.parking_distance)) + 0.5)
        self.backward_speed = -self.forward_speed

        variable_desired_distance_delta = self.desired_distance_delta
        if self.in_happy:
            variable_desired_distance_delta *= 2

        self.in_happy = False
        if abs(distance_to_cone - self.parking_distance) < variable_desired_distance_delta and abs(angle_to_cone) < self.approach_angle_delta:
            self.in_happy = True
            self.get_logger().info(f'happy distance of {distance_to_cone - self.parking_distance}')
            drive_cmd.drive.speed = 0.0
            drive_cmd.drive.steering_angle = 0.0        

            
        elif distance_to_cone > self.parking_distance:
            self.get_logger().info(f'too far away: {distance_to_cone - self.parking_distance}')
            if self.relative_y > 0: # left turn
                cone_to_rad_center = np.sqrt((self.relative_x)**2 + (self.relative_y - self.car_min_turning_radius)**2)
            else: # if  self.relative_y <= 0: # right turn
                cone_to_rad_center = np.sqrt((self.relative_x)**2 + (self.relative_y + self.car_min_turning_radius)**2)
            
            tangent_length = np.sqrt(cone_to_rad_center**2 - self.car_min_turning_radius**2)

            if tangent_length < self.parking_distance:
                # need to reverse 1st
                drive_cmd.drive.steering_angle = -angle_to_cone
                drive_cmd.drive.speed = self.backward_speed
                # Then can go forward 
            else:
                drive_cmd.drive.steering_angle = angle_to_cone
                drive_cmd.drive.speed = self.forward_speed 
            
        
        else: # if distance_to_cone < self.parking_distance or good distance but not facing the cone:
            self.get_logger().info(f'too close: {distance_to_cone - self.parking_distance}')
            drive_cmd.drive.steering_angle = -angle_to_cone
            drive_cmd.drive.speed = self.backward_speed
        
        self.last_wheel_angle = drive_cmd.drive.steering_angle
        self.last_car_velocity = drive_cmd.drive.speed

        for _ in range(1):
            self.drive_pub.publish(drive_cmd)



        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        #################################

        # YOUR CODE HERE
        # Populate error_msg with relative_x, relative_y, sqrt(x^2+y^2)

        error_msg.x_error = self.relative_x - self.parking_distance
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(self.relative_x**2 + self.relative_y**2) - self.parking_distance
        #################################
        
        self.error_pub.publish(error_msg)

def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
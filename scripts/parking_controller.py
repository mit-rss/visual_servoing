#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from purepursuit import *

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    PARK_DIST = rospy.get_param("parking_controller/parking_distance") # meters; try playing with this number!
    VEL = rospy.get_param("parking_controller/velocity")  

    # PP Stuff
    LIDAR_TO_BASE_AXEL = -0.35 # Temporary parameter
    LOOKAHEAD_DISTANCE = PARK_DIST / 1.5 # Should be smaller than parking distance
    L = 0.375

    # Controller Stuff
    GOOD_EPS = rospy.get_param("parking_controller/goal_range")
    PARK_TOL = rospy.get_param("parking_controller/y_tolerance")  
    ANG_EPS = abs(np.arctan(PARK_TOL / PARK_DIST))

    def __init__(self):

        DRIVE_TOPIC = rospy.get_param("~drive_topic")    
        rospy.Subscriber("/relative_cone", ConeLocation, self.relative_cone_callback)
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error", ParkingError, queue_size=10)
        
        self.relative_x = 0
        self.relative_y = 0
        self.backward = False


    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        
        # Are we here
        if abs(self.distance() - self.PARK_DIST) <= self.GOOD_EPS and abs(self.angle()) <= self.ANG_EPS:
            
            ## DONE
            eta = 0
            vel = 0

        else:

            ## Do we need to define a line and stick with it, or can we re-define every timestep?

            waypoints = np.array(([self.LIDAR_TO_BASE_AXEL, 0], \
                [self.LIDAR_TO_BASE_AXEL + self.relative_x, self.relative_y]))

            eta, vel = purepursuit(self.LOOKAHEAD_DISTANCE, self.L, self.VEL, 
                self.LIDAR_TO_BASE_AXEL, 0, 0, waypoints)
            
            # Backward PP if we are close
            if self.distance() < self.PARK_DIST - self.GOOD_EPS:
                self.backward = True

            # Forward PP if we are far
            if self.distance() > self.PARK_DIST + self.GOOD_EPS:
                self.backward = False

            # Reverse PP
            if self.backward:
                eta = -1 * eta
                vel = -1 * vel
                
        self.send_drive(eta, vel)
        self.error_publisher()


    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = self.distance()
        
        self.error_pub.publish(error_msg)


    def send_drive(self, eta, vel):
        """
        Helper function
        Sends AckermannDrive msg w/ eta, vel values
        """
        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        msg.drive.steering_angle = eta
        msg.drive.speed = vel
        self.drive_pub.publish(msg)

    def distance(self):
        return np.sqrt(self.relative_x**2 + self.relative_y**2)

    def angle(self):
        return np.arctan(self.relative_y / self.relative_x)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

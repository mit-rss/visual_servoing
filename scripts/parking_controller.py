#!/usr/bin/env python

import rospy
import numpy as np

from visual_servoing.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped
from purepursuit.py import *

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """

    DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
    
    # PP Stuff
    LIDAR_TO_BASE_AXEL = -0.15 # Temporary parameter
    LOOKAHEAD_DISTANCE = 0.5 # Should be smaller than parking distance
    L = 0.25

    # Controller Stuff
    EPS = .1 # Buffer of "ok" locations
    ANG_EPS = np.pi / 6 # In radians

    def __init__(self):

        rospy.Subscriber("/relative_cone", ConeLocation, self.relative_cone_callback)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error", ParkingError, queue_size=10)

        self.parking_distance = .75 # meters; try playing with this number!
        self.veloity = 1
        self.relative_x = 0
        self.relative_y = 0

    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        # Are we here
        if abs(self.relative_x - .75) <= self.EPS and abs(self.relative_y - .75) <= self.EPS \
            and abs(np.tan(self.relative_y / self.relative_x)) <= self.ANG_EPS:
            
            ## DONE
            eta = 0
            vel = 0
            

        # PP
        elif self.relative_x > self.parking_distance and \
            np.sqrt(self.relative_x**2 + self.relative_y**2) > self.parking_distance:

            # PP to that location
            waypoints = np.array([0, 0],[self.relative_x, self.relative_y]) ## Current location, Cone location
            eta, vel = purepursuit(self.LOOKAHEAD_DISTANCE, self.L, self.velocity, 0, 0, \
                self.LIDAR_TO_BASE_AXEL, waypoints)

        else: # Backup control logic (## Room to make this interesting -- could derive backwards PP)
            
            # # Bang-bang backwards 
            # if self.relative_y > 0
            #     eta = -.025 # Some eta nominal
            # else:
            #     seta = 0.025

            eta = 0

            vel = -1 * self.velocity

        msg.drive.steering_angle = eta
        msg.drive.speed = vel
        self.drive_pub.publish(drive_cmd)
        self.error_publisher()

    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        error_msg = ParkingError()

        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt(self.relative_x**2 + self.relative_y**2)
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

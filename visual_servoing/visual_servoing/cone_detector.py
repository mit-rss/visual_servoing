#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point #geometry_msgs not in CMake file
from vs_msgs.msg import ConeLocationPixel

# import your color segmentation algorithm; call this function in ros_image_callback!
# from computer_vision.color_segmentation import cd_color_segmentation


class ConeDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_cone_px (ConeLocationPixel) : the coordinates of the cone in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("cone_detector")
        # toggle line follower vs cone parker
        self.LineFollower = False

        # Subscribe to ZED camera RGB frames
        self.cone_pub = self.create_publisher(ConeLocationPixel, "/relative_cone_px", 10)
        self.debug_pub = self.create_publisher(Image, "/cone_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Cone Detector Initialized")

    def image_callback(self, image_msg):
        # Apply your imported color segmentation function (cd_color_segmentation) to the image msg here
        # From your bounding box, take the center pixel on the bottom
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_cone_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # YOUR CODE HERE
        # detect the cone and publish its
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        img = np.array(image, dtype=np.uint8)

        #cv2.namedWindow("original", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("hsv", cv2.WINDOW_NORMAL)
        #cv2.namedWindow("mask", cv2.WINDOW_NORMAL)
        # cv2.namedWindow("trackbars", 0)
        # create_trackbars()
        # while True:
            
        H_min ,S_min ,V_min ,H_max ,S_max ,V_max = [5,120,20,40,255,255] #[5,150,70,30,255,255]#get_trackbar_values()#
        frame_to_mask = cv2.cvtColor(img , cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(frame_to_mask , (H_min , S_min , V_min), (H_max , S_max , V_max))
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.dilate(mask, kernel, iterations = 2)
        mask_height, mask_width = mask.shape[:2]
        visible_bar_height = 40
        mask = cv2.rectangle(mask, (0, 300), (mask_width-1, mask_height-1), (0,0,0), -1)
        mask = cv2.rectangle(mask, (0, 0), (mask_width-1, 300-visible_bar_height), (0,0,0), -1)
            
        contr, heir = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if (len(contr) != 0):
            c = max(contr, key = cv2.contourArea)
            x, y, w, h = cv2.boundingRect(c)
        # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
        

            
        # cv2.imshow("original", img)
        # cv2.imshow("hsv", frame_to_mask)
        # cv2.imshow("mask", mask)
        # cv2.waitKey(0)

            cone_location = ConeLocationPixel()
            cone_location.u = (x+w/2.0)
            cone_location.v = float(y+h)
            self.cone_pub.publish(cone_location)
    
            new_img = cv2.bitwise_and(img, img, mask=mask)
            cv2.rectangle(new_img,(x,y),(x+w,y+h),(0,255,0),2)

            debug_msg = self.bridge.cv2_to_imgmsg(new_img, "bgr8")
            self.debug_pub.publish(debug_msg)

def main(args=None):
    rclpy.init(args=args)
    cone_detector = ConeDetector()
    rclpy.spin(cone_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

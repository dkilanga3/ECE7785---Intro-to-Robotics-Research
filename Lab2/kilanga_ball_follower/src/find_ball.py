#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Sat Sep 15 14:46:02 2018

@author: dkilanga
"""

# Python libs
import sys
import imutils

# Numpy and scipy
import numpy as np

# OpenCV
import cv2

# ROS libs
import rospy

# ROS Messages
from geometry_msgs.msg import Point
from sensor_msgs.msg import CompressedImage


class find_ball:
    
    def __init__(self):
        '''Initialize ROS publisher and ROS subscriber'''
        
        # Subsribed Topic
        self.image_subscribed = rospy.Subscriber("/raspicam_node/image/compressed",CompressedImage,self.callback)
        
        # Publish Topics
        self.point_publisher = rospy.Publisher("/output/ball_center_pixel/", Point, queue_size=1)
        self.image_publisher = rospy.Publisher("/output/image_raw/compressed/processed_frame/compressed", CompressedImage)


        self.pinkMinBound = np.array([140, 50, 50])
        self.pinkMaxBound = np.array([180, 255, 255])

    def callback(self, input_frame):
        frame = np.fromstring(input_frame.data, np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        frame = imutils.resize(frame, width=int(frame.shape[1]/2))
        output = frame.copy()
        
        # Blure the frame
        frame = cv2.GaussianBlur(frame, (11,11),0)
        
         # Change color space from RGB to HSV
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Mask all other colots but pink
        mask = cv2.inRange(hsv_frame, self.pinkMinBound, self.pinkMaxBound)
        
        # Blure mask
        mask = cv2.GaussianBlur(mask, (11,11),0)
        mask = cv2.erode(mask, None, iterations=3)
        mask = cv2.dilate(mask, None, iterations=3)
        mask = cv2.GaussianBlur(mask, (11,11),0)
        
        # Find circles with hough transform
        circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 90, param1=70, param2=30, \
                                   minRadius=0)
        
        #Create Point to be published
        ball_center_pixel = Point()
        # Operations to select only the most probable circle and find its center
        if circles is not None:
            circles = np.round(circles[0,:]).astype("int")
            moments = cv2.moments(mask)
            x_center = moments['m10']/moments['m00']
            y_center = moments['m01']/moments['m00']
            for (x, y, r) in circles:
                cv2.circle(output, (x, y), r, (0, 255, 0), 4)
                cv2.circle(output, (int(x_center),int(y_center)), 5, (0,0,255), -1)
            ball_center_pixel.x = x_center
            ball_center_pixel.y = y_center
            ball_center_pixel.z = 0
            self.point_publisher.publish(ball_center_pixel)
        else:
            ball_center_pixel.x = 80
            ball_center_pixel.y = 0
            ball_center_pixel.z = 0
            self.point_publisher.publish(ball_center_pixel)
            
#        cv2.imshow('Lab1Window', output)
        cv2.waitKey(1) & 0xFF == ord('q')
        
        # Create CompressedImage to be published for debugging
        frame_debug = CompressedImage()
        frame_debug.header.stamp = rospy.Time.now()
        frame_debug.format = "jpeg"
        frame_debug.data = np.array(cv2.imencode('.jpeg', output)[1]).tostring()
        self.image_publisher.publish(frame_debug)
        
        

def main(args):
    '''Initialize and cleanup ROS node'''
    fb = find_ball()
    rospy.init_node('find_ball', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Find Ball node"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)
    
    
    
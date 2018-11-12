#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Tue Sep 18 00:52:02 2018

@author: dkilanga
"""

# Python libs
import sys

# Numpy and scipy
import numpy as np

# OpenCV
import cv2

# ROS libs
import rospy

# ROS Messages
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import CompressedImage


class drive_wheels:
    
    def __init__(self):
        '''Initialize ROS publisher and ROS subscriber'''
        
        # Suscribed Topic
        self.point_subscribed = rospy.Subscriber("/output/ball_center_pixel/",Point,self.callback)
        
        # Publish Topics
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        self.max_angular_velocity = 0.6
        self.min_angular_velocity = 0.4
        self.angular_velocity = 0
        self.camera_center_x = 80
        self.velocity_to_send = Twist()
        self.velocity_to_send.linear.x = 0
        self.velocity_to_send.linear.y = 0
        self.velocity_to_send.linear.z = 0
        self.velocity_to_send.angular.x = 0
        self.velocity_to_send.angular.y = 0
        self.velocity_to_send.angular.z = 0
                
    def callback(self, input_point):
        
        self.speed_coefficient = self.camera_center_x/self.max_angular_velocity/4
        self.angular_velocity = -(input_point.x - self.camera_center_x)/self.speed_coefficient
        
        if (self.angular_velocity >= -(self.min_angular_velocity/2)) and (self.angular_velocity <= (self.min_angular_velocity/2)):
            self.velocity_to_send.angular.z = 0
        elif self.angular_velocity >= self.max_angular_velocity:
            self.velocity_to_send.angular.z = self.max_angular_velocity
        elif self.angular_velocity <= -self.max_angular_velocity:
            self.velocity_to_send.angular.z = -self.max_angular_velocity
        else:
            self.velocity_to_send.angular.z = self.angular_velocity
        self.velocity_publisher.publish(self.velocity_to_send)
        
def main(args):
    '''Initialize and cleanup ROS node'''
    rospy.init_node('drive_wheels', anonymous=True)
    dw = drive_wheels()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Drive Wheels node"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)
        
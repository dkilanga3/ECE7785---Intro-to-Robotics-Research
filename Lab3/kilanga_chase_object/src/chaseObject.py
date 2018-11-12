#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Oct  4 16:03:29 2018

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
from sensor_msgs.msg import LaserScan

class chaseObject:
    
    def __init__(self):
        '''Initialize ROS publisher and ROS subscriber'''
         # Subsribed Topic
        self.position_subscribed = rospy.Subscriber("/object/position", Twist, self.callback)
        
        # Publish Topic
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=5)
        
        self.max_angular_velocity = 0.6
        self.min_angular_velocity = 0.4
        self.angular_velocity = 0
        
        self.max_linear_velocity = 0.05
        self.min_linear_velocity = 0.04
        self.linear_velocity = 0
        
        self.Derivator_angular = 0
        self.Integrator_angular = 0
        self.Integrator_max_angular=10
        self.Integrator_min_angular=-10
        
        self.Derivator_linear = 0
        self.Integrator_linear = 0
        self.Integrator_max_linear=10
        self.Integrator_min_linear=-10
        
        self.velocity_to_send = Twist()
        self.velocity_to_send.linear.x = 0
        self.velocity_to_send.linear.y = 0
        self.velocity_to_send.linear.z = 0
        self.velocity_to_send.angular.x = 0
        self.velocity_to_send.angular.y = 0
        self.velocity_to_send.angular.z = 0
        
        self.field_of_view = 62.2
        
        
    def callback(self,angular_pos_and_dist):
        self.camera_center_x = angular_pos_and_dist.linear.y/2
        self.degree_per_pixel = self.field_of_view/angular_pos_and_dist.linear.y
        self.set_angle = self.camera_center_x * self.degree_per_pixel
        self.set_distance = 0.5
        self.current_angle = angular_pos_and_dist.angular.z
        self.current_distance = angular_pos_and_dist.linear.x
        print "angle: ", self.current_angle, " dist ", self.current_distance
        
        if angular_pos_and_dist.linear.x != 999:
            self.pid_angular()
            if (self.angular_velocity >= -(self.min_angular_velocity/2)) and (self.angular_velocity <= (self.min_angular_velocity/2)):
                self.velocity_to_send.angular.z = 0.0
            elif self.angular_velocity >= self.max_angular_velocity:
                self.velocity_to_send.angular.z = self.max_angular_velocity
            elif self.angular_velocity <= -self.max_angular_velocity:
                self.velocity_to_send.angular.z = -self.max_angular_velocity
            else:
                self.velocity_to_send.angular.z = self.angular_velocity
                
            #if (self.current_angle >= 26) and (self.current_angle <= 35):
            self.pid_linear()
            if (self.linear_velocity >= -(self.min_linear_velocity/2)) and (self.linear_velocity <= (self.min_linear_velocity/2)):
                self.velocity_to_send.linear.x = 0.0
            if np.isnan(self.linear_velocity):
                self.velocity_to_send.linear.x = 0.0
            elif self.linear_velocity >= self.max_linear_velocity:
                self.velocity_to_send.linear.x = self.max_linear_velocity
            elif self.linear_velocity <= -self.max_linear_velocity:
                self.velocity_to_send.linear.x = -self.max_linear_velocity
            else:
                self.velocity_to_send.linear.x = self.linear_velocity
        else:
            self.velocity_to_send.angular.z = 0.0
            self.velocity_to_send.linear.x = 0.0
        self.velocity_publisher.publish(self.velocity_to_send)
            
    def pid_angular(self):
        # Proportional gain of PID controller
        self.k_p_angular = 1/(self.camera_center_x/self.max_angular_velocity/8)
        self.angular_velocity = self.k_p_angular * (self.set_angle-self.current_angle)
        
    def pid_linear(self):
        # Proportional gain of PID controller
        error = self.set_distance-self.current_distance
        #self.k_p_linear = self.max_linear_velocity*(1-np.exp(-abs(error)**2))/abs(error)
        self.k_p_linear = 0.5
        self.linear_velocity = -self.k_p_linear * (error)
        print "linear velocity: ", self.linear_velocity, "error :", error

def main(args):
    '''Initialize and cleanup ROS node'''
    rospy.init_node('chaseObject', anonymous=True)
    cO = chaseObject()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Chase Object node"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)    

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Oct  3 23:05:27 2018

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

class getObjectRange:
    
    def __init__(self):
        '''Initialize ROS publisher and ROS subscriber'''
         # Subsribed Topic
        self.lidar_subscribed = rospy.Subscriber("/scan", LaserScan, self.callback_lidar)
        self.point_location_subscribed = rospy.Subscriber('/imageLocation', Point, self.callback_point)
        # Publish Topic
        self.position_publisher = rospy.Publisher("/object/position", Twist, queue_size=1)
        

        self.angular_position_and_distance = Twist()
        self.angular_position_and_distance.linear.x = 0
        self.angular_position_and_distance.linear.y = 0
        self.angular_position_and_distance.linear.z = 0
        self.angular_position_and_distance.angular.x = 0
        self.angular_position_and_distance.angular.y = 0
        self.angular_position_and_distance.angular.z = 0
        
        self.point_check = False
        self.field_of_view = 62.2
        self.lidar_err = 0.005
        
    def callback_point(self,point):
        self.degree_per_pixel = self.field_of_view/point.z
        #print "My x pixel is: ", point.x
#        while not rospy.is_shutdown():
        if point.x != 999:
            current_angle = point.x * self.degree_per_pixel
            self.current_angle = np.rad2deg(np.arctan2(np.sin(np.deg2rad(current_angle)),np.cos(np.deg2rad(current_angle))))
            self.point_check = True
            self.angular_position_and_distance.linear.y = point.z
            self.angular_position_and_distance.angular.z = self.current_angle
            #self.get_lidar_scan()
        else:
            self.point_check = False
            self.angular_position_and_distance.linear.x = 999
            self.angular_position_and_distance.linear.y = point.z
            self.angular_position_and_distance.angular.z = 999
        self.position_publisher.publish(self.angular_position_and_distance)
    
    def callback_lidar(self,lidar):
        if self.point_check:
            #print lidar.ranges
            distances = []
            if self.current_angle >= 31:
                lidar_angle = int(360-(self.current_angle-31))
                angles_of_interest = [lidar_angle,lidar_angle-1,lidar_angle-2,lidar_angle-3, lidar_angle-4, lidar_angle-5,(lidar_angle+1)%360,(lidar_angle+2)%360,(lidar_angle+3)%360,(lidar_angle+4)%360,(lidar_angle+5)%360]
                print "angles:  ", angles_of_interest
                for i in angles_of_interest:
                    #print "angle", i
                    if (lidar.ranges[i] >= self.lidar_err) and (abs(lidar.ranges[lidar_angle]-lidar.ranges[i])<=0.05 ):
                        distances.append(lidar.ranges[i])
                print "distances: ", distances
                self.angular_position_and_distance.linear.x = np.mean(distances)
            elif self.current_angle < 31:
                lidar_angle = int(31-self.current_angle)
                angles_of_interest = [lidar_angle,lidar_angle+1,lidar_angle+2,lidar_angle+3,lidar_angle+4,lidar_angle+5,lidar_angle-1,lidar_angle-2,lidar_angle-3,lidar_angle-4,lidar_angle-5]
                print "angles:  ", angles_of_interest
                for i in angles_of_interest:
                    #print "angle", i
                    if i < 0:
                        i = 360+i
                    if (lidar.ranges[i] >= self.lidar_err) and (abs(lidar.ranges[lidar_angle]-lidar.ranges[i])<=0.05 ):
                        distances.append(lidar.ranges[i])
                print "distances:  ", distances
                self.angular_position_and_distance.linear.x = np.mean(distances)
            
def main(args):
    '''Initialize and cleanup ROS node'''
    rospy.init_node('getObjectRange', anonymous=True)
    objRange = getObjectRange()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Get Object Range node"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)
            
        
    
        

#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 19 15:06:48 2018

@author: dkilanga
"""

# Python libs
import sys
from operator import itemgetter

# Numpy and scipy
import numpy as np

# ROS libs
import rospy

# ROS Messages
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class getObjectRange:
    
    def __init__(self):
        '''Initialize ROS publisher and ROS subscriber'''
         # Subsribed Topic
        self.lidar_subscribed = rospy.Subscriber("/scan", LaserScan, self.callback_lidar)
        # Publish Topic
        self.position_publisher = rospy.Publisher("/position_and_orientation", Twist, queue_size=1)
        
        self.position_orientation_obstacle = Twist()
        self.position_orientation_obstacle.linear.x = 0
        self.position_orientation_obstacle.linear.y = 0
        self.position_orientation_obstacle.linear.z = 0
        self.position_orientation_obstacle.angular.x = 0
        self.position_orientation_obstacle.angular.y = 0
        self.position_orientation_obstacle.angular.z = 0
        
        self.lidar_error = 0.005
        
    def callback_lidar(self, lidar):
        array_lidar = np.array(lidar.ranges)
        arr_lidar = np.array(lidar.ranges)
        array_lidar = np.roll(array_lidar,10)[:20]
        indices =np.array( range(350,360)+range(0,10))
        non_zero_lidar = np.where(array_lidar > 0.09)
        zero_lidar = np.where(array_lidar < 0.09)
        array_lidar = array_lidar[non_zero_lidar]
        indices = indices[non_zero_lidar]
        if len(indices) > 0:
            range_of_interest = np.min(array_lidar)
            index_of_interest = np.argmin(array_lidar)
            angle = indices[index_of_interest]
            if (index_of_interest == 0) and len(indices) > 3:
                self.position_orientation_obstacle.linear.x = np.mean(array_lidar[index_of_interest:index_of_interest+3])
                self.position_orientation_obstacle.angular.z = indices[index_of_interest]
            elif (index_of_interest == (len(indices)-1)) and len(indices) > 3:
                self.position_orientation_obstacle.linear.x = np.mean(array_lidar[index_of_interest-2:])
                self.position_orientation_obstacle.angular.z = indices[index_of_interest]
            elif len(indices) > 3:
                self.position_orientation_obstacle.linear.x = np.mean(array_lidar[index_of_interest-1:index_of_interest+2])
                self.position_orientation_obstacle.angular.z = indices[index_of_interest]
            else:
                self.position_orientation_obstacle.linear.x = range_of_interest
                self.position_orientation_obstacle.angular.z = indices[index_of_interest]
        else:
            self.position_orientation_obstacle.linear.x = 10.0
            self.position_orientation_obstacle.angular.z = 0


        self.position_publisher.publish(self.position_orientation_obstacle)
        
def main(args):
    '''Initialize and cleanup ROS node'''
    rospy.init_node('getObjectRange', anonymous=True)
    objRange = getObjectRange()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Get Object Range node"
    
if __name__ == '__main__':
    main(sys.argv)
            
        

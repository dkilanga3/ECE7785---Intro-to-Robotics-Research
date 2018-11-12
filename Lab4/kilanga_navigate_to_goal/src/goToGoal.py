#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 22 19:42:51 2018

@author: dkilanga
"""

# Python libs
import sys
import os
import time
import signal

# Numpy and scipy
import numpy as np

# ROS libs
import rospy

# ROS Messages
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry

class goToGoal:
    
    def __init__(self):
        rospy.init_node('goToGoal', anonymous = True)
        '''Initialize ROS publisher and ROS subscriber'''
        # Subscribed Topic
        self.position_subscribed = rospy.Subscriber("/position_and_orientation", Twist, self.callback_pos_orient)
        self.odometry_subscribed = rospy.Subscriber("/odom", Odometry, self.callback_odom)
        
        # Publish Topic
        self.command_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.wayPoints = []
        self.readWayPoints()
        self.Init = True
        self.Init_pos = Point()
        self.globalPos = Point()
        self.globalObstaclePos = Point()
        
        self.distance_to_keep = 0.25
        self.epsilon = 0.07
        self.mode = "goGoal"
        
        self.X_goal = np.zeros(2)
        self.X_robot = np.zeros(2)
        self.robot_control = np.zeros(2)
        self.X_obstacle = np.array([10.0,10.0])
        
        self.theta_desired_goal = 0
        self.theta_desired_fwc = 0
        self.theta_desired_fwcc = 0
        self.count = 0
        self.switch_count = 0
        self.globalAng = 0
        
        self.goal_1_reached = False
        self.goal_2_reached = False
        self.goal_3_reached = False
        
        self.command_to_send = Twist()
        self.command_to_send.linear.x = 0
        self.command_to_send.linear.y = 0
        self.command_to_send.linear.z = 0
        self.command_to_send.angular.x = 0
        self.command_to_send.angular.y = 0
        self.command_to_send.angular.z = 0
        
        self.operate()
        try:
            rospy.spin()
        except keyboadInterrupt:
            print "Shutting down ROS Go To Goal node"
    
    
    def readWayPoints(self):     
        list_of_lists = [[1.45,0],[1.45,1.5],[0,1.5]]
        wayPoints = np.array(list_of_lists)
        
        wayPoint1 = Twist()
        wayPoint1.linear.x = wayPoints[0][0]
        wayPoint1.linear.y = wayPoints[0][1]
        wayPoint1.linear.z = 0
        wayPoint1.angular.x = 0
        wayPoint1.angular.y = 0
        wayPoint1.angular.z = 0
        self.wayPoints.append(wayPoint1)
        
        wayPoint2 = Twist()
        wayPoint2.linear.x = wayPoints[1][0]
        wayPoint2.linear.y = wayPoints[1][1]
        wayPoint2.linear.z = 0
        wayPoint2.angular.x = 0
        wayPoint2.angular.y = 0
        wayPoint2.angular.z = 0
        self.wayPoints.append(wayPoint2)
        
        wayPoint3 = Twist()
        wayPoint3.linear.x = wayPoints[2][0]
        wayPoint3.linear.y = wayPoints[2][1]
        wayPoint3.linear.z = 0
        wayPoint3.angular.x = 0
        wayPoint3.angular.y = 0
        wayPoint3.angular.z = 0
        self.wayPoints.append(wayPoint3)
        
        
    def callback_odom(self,Odom):
        
        position = Odom.pose.pose.position
        
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.Init:
            #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
            
    
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])        
    
        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        self.X_robot[0] = self.globalPos.x
        self.X_robot[1] = self.globalPos.y
        
    def operate(self):
        while True:
            if (self.goal_1_reached == False) and (self.goal_2_reached == False) and (self.goal_3_reached == False):
                self.X_goal[0] = (self.wayPoints[0]).linear.x
                self.X_goal[1] = (self.wayPoints[0]).linear.y
                self.goGoal_controller(1)
            if (self.goal_1_reached == True) and (self.goal_2_reached == False) and (self.goal_3_reached == False):
                self.X_goal[0] = (self.wayPoints[1]).linear.x
                self.X_goal[1] = (self.wayPoints[1]).linear.y
                self.goGoal_controller(2)
            if (self.goal_1_reached == True) and (self.goal_2_reached == True) and (self.goal_3_reached == False):
                self.X_goal[0] = (self.wayPoints[2]).linear.x
                self.X_goal[1] = (self.wayPoints[2]).linear.y
                self.goGoal_controller(3)
            self.avoidObstacle_controller()
            self.followWallC_controller()
            self.followWallCC_controller()
            self.switching_logic()
            self.check_progress()
            self.transform_unicyle()
            self.command_to_send.linear.x = self.robot_control[0]
            self.command_to_send.angular.z = self.robot_control[1]
            self.command_publisher.publish(self.command_to_send)
            rospy.sleep(.1)
            signal.signal(signal.SIGINT, self.signal_handler)
        
    def switching_logic(self):
        
        self.distance_to_object = np.linalg.norm(self.X_robot - self.X_obstacle)
                
        if(self.mode == "goGoal") and (self.distance_to_object <= (self.distance_to_keep+self.epsilon)) and (self.u_GTG.T.dot(self.u_FWC) > 0):
            
            self.X_robot_at_switch = np.copy(self.X_robot)
            self.mode = "followWallC"
            self.switch_time = rospy.Time.now()
            self.switch_count+=1

            
        if(self.mode == "followWallC") and (self.distance_to_object < (self.distance_to_keep-self.epsilon)):
            self.mode = "avoidObstacle"
            
        if(self.mode == "avoidObstacle") and (self.distance_to_object >= (self.distance_to_keep-self.epsilon)) and (self.u_GTG.T.dot(self.u_FWC) > 0):
            self.X_robot_at_switch = np.copy(self.X_robot)
            self.mode = "followWallC"
            self.switch_time = rospy.Time.now()
            self.switch_count+=1

            
        if(self.mode == "avoidObstacle") and (self.distance_to_object >= (self.distance_to_keep-self.epsilon)) and (self.u_GTG.T.dot(self.u_FWCC) > 0):
            self.X_robot_at_switch = np.copy(self.X_robot)
            self.mode = "followWallCC"
            self.switch_time = rospy.Time.now()
            self.switch_count+=1

            
        if(self.mode == "followWallCC") and (self.distance_to_object < (self.distance_to_keep-self.epsilon)):
            self.mode = "avoidObstacle"
            
        if((self.mode == "followWallC")or(self.mode == "followWallCC")) and (self.u_GTG.T.dot(self.u_AO) > 0) and \
                 (np.linalg.norm(self.X_robot-self.X_goal) < (np.linalg.norm(self.X_robot_at_switch-self.X_goal)+0.9)) and\
                 (self.distance_to_object > (self.distance_to_keep+self.epsilon)) and ((rospy.Time.now()-self.switch_time)>= rospy.Duration(4)):
            self.mode = "goGoal"
            
        if(self.mode == "goGoal") and (self.distance_to_object < (self.distance_to_keep+self.epsilon)) and (self.u_GTG.T.dot(self.u_FWCC) > 0):
            self.X_robot_at_switch = np.copy(self.X_robot)
            self.mode = "followWallCC"
            self.switch_time = rospy.Time.now()
            self.switch_count+=1

                                                                              
        
        
    def callback_pos_orient(self,obstacle_range_orient):
        self.globalObstaclePos.x = self.globalPos.x + obstacle_range_orient.linear.x * np.cos(self.globalAng + np.deg2rad(obstacle_range_orient.angular.z))
        self.globalObstaclePos.y = self.globalPos.y + obstacle_range_orient.linear.x * np.sin(self.globalAng + np.deg2rad(obstacle_range_orient.angular.z))
        self.obstacle_range = obstacle_range_orient.linear.x
        self.obstacle_orient = obstacle_range_orient.angular.z
        self.X_obstacle[0] = self.globalObstaclePos.x
        self.X_obstacle[1] = self.globalObstaclePos.y

    def goGoal_controller(self,goal):
        
        k_GTG = 0.8
        self.u_GTG = k_GTG*(self.X_goal - self.X_robot)
        
        
    def avoidObstacle_controller(self):
        
        k_AO = 0.5
        self.u_AO = -k_AO*(self.X_obstacle - self.X_robot)
        
    def followWallC_controller(self):
        
        alpha = 0.5
        self.u_FWC = alpha*(self.rotationMatrix(-np.pi/2)).dot(self.u_AO)
        
    def followWallCC_controller(self):
        
        alpha = 0.5
        self.u_FWCC = alpha*(self.rotationMatrix(np.pi/2)).dot(self.u_AO)
    
    def rotationMatrix(self,theta):
        
        matrix = np.zeros((2,2))
        matrix[0][0] = np.cos(theta)
        matrix[0][1] = np.sin(theta)
        matrix[1][0] = -np.sin(theta)
        matrix[1][1] = np.cos(theta)
        
        return matrix
    
    def desired_heading(self):
        self.theta_desired_goal = np.arctan2((self.X_goal[1]-self.X_robot[1]),(self.X_goal[0]-self.X_robot[0]))
        self.theta_desired_fwc = np.arctan2(self.u_FWC[0],self.u_FWC[1])
        self.theta_desired_fwcc = np.arctan2(self.u_FWCC[0],self.u_FWCC[1])
        
        
    
    def transform_unicyle(self):
        self.desired_heading()
        self.k_p_angular = 1
        self.k_p_linear = 1.5
        if self.mode == "goGoal":
            angle_error = self.theta_desired_goal-self.globalAng
            angle_error = np.arctan2(np.sin(angle_error),np.cos(angle_error))
            self.angular_velocity = self.k_p_angular * (angle_error)
            if abs(angle_error) <= np.deg2rad(10):
                self.linear_velocity =self.k_p_linear * np.linalg.norm(self.u_GTG)
            else:
                self.linear_velocity = 0
            self.robot_control[0] = self.linear_velocity
            self.robot_control[1] = self.angular_velocity
            
        elif self.mode =="avoidObstacle":
            self.angular_velocity = 0
            self.linear_velocity = -self.k_p_linear * np.linalg.norm(self.u_AO)
            self.robot_control[0] = self.linear_velocity
            self.robot_control[1] = self.angular_velocity
        elif self.mode == "followWallC":
            self.angular_velocity = 0.5*self.k_p_angular * (self.theta_desired_fwc-self.globalAng)
            self.linear_velocity = 1.0*self.k_p_linear * np.linalg.norm(self.u_FWC)
            self.robot_control[0] = self.linear_velocity
            self.robot_control[1] = self.angular_velocity
            print "Follow Wall Clockwise heading: ", np.rad2deg(self.theta_desired_fwc), np.linalg.norm(self.u_FWC)
        elif self.mode == "followWallCC":
            self.angular_velocity = -0.5*self.k_p_angular * (self.theta_desired_fwcc-self.globalAng)
            self.linear_velocity = 1.0*self.k_p_linear * np.linalg.norm(self.u_FWCC)
            self.robot_control[0] = self.linear_velocity
            self.robot_control[1] = self.angular_velocity
            
    def check_progress(self):
        
        
        if(self.X_robot[0] >= 1.45) and (self.X_robot[0] <= 1.50) and (self.X_robot[1] >= -0.10) and (self.X_robot[1] <= 0.10) and (self.count == 0):
            self.goal_1_reached = True
            self.command_to_send.linear.x = 0
            self.command_to_send.angular.z = 0
            self.command_publisher.publish(self.command_to_send)
            rospy.sleep(3)
            self.count += 1
        if(self.X_robot[0] >= 1.45) and (self.X_robot[0] <= 1.6) and (self.X_robot[1] >= 1.45) and (self.X_robot[1] <= 1.65) and (self.count == 1):
            self.goal_2_reached = True
            self.command_to_send.linear.x = 0
            self.command_to_send.angular.z = 0
            self.command_publisher.publish(self.command_to_send)
            rospy.sleep(3)
            self.count += 1
        if(self.X_robot[0] >= -0.20) and (self.X_robot[0] <= 0.20) and (self.X_robot[1] >= 1.30) and (self.X_robot[1] <= 1.70) and (self.count == 2):
            self.goal_3_reached = True
            self.command_to_send.linear.x = 0
            self.command_to_send.angular.z = 0
            self.command_publisher.publish(self.command_to_send)
            rospy.sleep(3)
            self.count += 1
            
    def rotate_90_deg(self):
        init = self.globalAng
        desired = init + np.deg2rad(90)
        desired = np.arctan2(np.sin(desired), np.cos(desired))
        error = desired - init
        k_p = 0.5
        while abs(error) > np.deg2rad(3):
            error = desired - self.globalAng
            error = np.arctan2(np.sin(error),np.cos(error))
            angular_command = k_p * (error)
            self.command_to_send.linear.x = 0
            self.command_to_send.angular.z = angular_command
            self.command_publisher.publish(self.command_to_send)
            time.sleep(.5)
            signal.signal(signal.SIGINT, self.signal_handler)
            
    def signal_handler(self,sig1=None,sig2=None):
        self.command_to_send.angular.z = 0
        self.command_to_send.linear.x = 0
        self.command_publisher.publish(self.command_to_send)
        sys.exit()

def main(args):
    '''Initialize and cleanup ROS node'''
    
    try:
        goGoal = goToGoal()
    except rospy.ROSInterruptException:
        pass
   
    
if __name__ == '__main__':
    main(sys.argv)    
        
            

            
        

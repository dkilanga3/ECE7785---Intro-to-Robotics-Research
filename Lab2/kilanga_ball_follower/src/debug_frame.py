#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Sep 19 00:06:22 2018

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
from sensor_msgs.msg import CompressedImage

class debug_frame:
    def __init__(self):
        '''Initialize ROS publisher and ROS subscriber'''
        
        # Subsribed Topic
        self.image_subscribed = rospy.Subscriber("/output/image_raw/compressed/processed_frame/compressed",CompressedImage,self.callback,queue_size=1)
    
    def callback(self, input_frame):
        frame = np.fromstring(input_frame.data, np.uint8)
        frame = cv2.imdecode(frame, cv2.IMREAD_COLOR)
        frame = imutils.resize(frame, width=int(frame.shape[1]*2))
        
        cv2.imshow('Lab1Window', frame)
        cv2.waitKey(1) & 0xFF == ord('q')
        
def main(args):
    '''Initialize and cleanup ROS node'''
    df = debug_frame()
    rospy.init_node('debug_frame', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Debug Frame node"
    cv2.destroyAllWindows()
    
if __name__ == '__main__':
    main(sys.argv)
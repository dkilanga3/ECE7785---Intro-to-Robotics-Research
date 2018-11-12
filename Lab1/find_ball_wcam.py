#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Thu Aug 30 21:51:30 2018

@author: dkilanga
"""

import numpy as np
import cv2
import imutils

cap = cv2.VideoCapture(0)

pinkMinBound = np.array([140, 50, 50])
pinkMaxBound = np.array([180, 255, 255])

cv2.namedWindow('Lab1Window', cv2.WINDOW_AUTOSIZE)

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # Resize the frame
    frame = imutils.resize(frame, width=600)
#    frame = imutils.resize(frame, width=int(frame.shape[1]/2))
#    print frame.shape

    # Blure the frame
    frame = cv2.GaussianBlur(frame, (11,11),0)
    
    # Change color space from RGB to HSV
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Mask all other colots but pink
    mask = cv2.inRange(hsv_frame, pinkMinBound, pinkMaxBound)
    # Blure mask
    mask = cv2.GaussianBlur(mask, (11,11),0)
    mask = cv2.erode(mask, None, iterations=3)
    mask = cv2.dilate(mask, None, iterations=3)
    mask = cv2.GaussianBlur(mask, (11,11),0)
    output = frame.copy()
    

    
    circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, 1, 90, param1=70, param2=30, \
                               minRadius=0)
    print circles
    if circles is not None:
        circles = np.round(circles[0,:]).astype("int")
        moments = cv2.moments(mask)
        x_center = moments['m10']/moments['m00']
        y_center = moments['m01']/moments['m00']
        for (x, y, r) in circles:
            cv2.circle(output, (x, y), r, (0, 255, 0), 4)
            cv2.circle(output, (int(x_center),int(y_center)), 5, (0,0,255), -1)

    

    cv2.imshow('Lab1Window', output)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
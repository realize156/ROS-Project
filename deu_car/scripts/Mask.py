#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy
import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

class Mask:
    def __init__(self):
        self.top = 0
        self.bot = 0
        self.left = 0
        self.right = 0
        self.type = ""
        self.bridge = cv_bridge.CvBridge()
        #self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.mask_callback)

    def mask_callback(self,msg, h, b, l, r, s):
        self.top = h
        self.bot = b
        self.left = l
        self.right = r
        self.type = s

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        if self.type == "yellow":#left yellow line
            lower = numpy.array([25, 20, 50])
            upper = numpy.array([35, 255, 255])
        elif self.type == "white":#left white line & stop line
            lower = numpy.array([0, 0, 0])
            upper = numpy.array([0, 0, 255])
        elif self.type == "red":#blocking bar
            lower = numpy.array([0, 200, 0])
            upper = numpy.array([0, 255, 255])

        mask = cv2.inRange(hsv, lower, upper)

        h, w, d = image.shape
        mask[0:self.top, 0:w] = 0
        mask[self.bot:h, 0:w] = 0
        mask[self.top:self.bot, 0:self.left] = 0
        mask[self.top:self.bot, self.right:w] = 0

        #Mask = cv2.moments(mask)
        return mask
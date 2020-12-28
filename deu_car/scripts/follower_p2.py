#!/usr/bin/env python

import cv2
import numpy

import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image


class Follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        # cv2.namedWindow("window",1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',Image,self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop',Twist,queue_size = 1)
        self.p2_image_pub = rospy.Publisher('camera/rgb/image_raw/p2',Image,queue_size = 1)
        self.mask_pub = rospy.Publisher('camera/rgb/image_raw/mask',Image,queue_size = 1)
        self.twist = Twist()
        self.count = 0
        self.counter = 0


    def image_callback(self,msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([25,20,50])
        upper_yellow = numpy.array([35,255,255])

        lower_white = numpy.array([0,0,180])
        upper_white = numpy.array([25,36,255])

        lower_white2 = numpy.array([0,0,190])
        upper_white2 = numpy.array([25,36,255])

        mask_left = cv2.inRange(hsv,lower_yellow,upper_yellow)
        mask_left2 = cv2.inRange(hsv,lower_white,upper_white)
        mask_middle = cv2.inRange(hsv, lower_white2, upper_white2)

        h,w,d = image.shape
        search_top = 3*h/4
        search_bot = 3*h/4+20
        search_across = w / 2

        #search_left = w / 3
        #search_right = w * 2 / 3

        mask_left[0:search_top, 0:w] = 0
        mask_left[search_bot:h, 0:w] = 0
        mask_left[0:h, search_across:w] = 0

        mask_left2[0:search_top, 0:w] = 0
        mask_left2[search_bot:h, 0:w] = 0
        mask_left2[0:h, search_across:w] = 0

        mask_middle[0:h - 120, 0:w] = 0
        mask_middle[0:h, 0:300] = 0
        mask_middle[0:h, w - 200:w] = 0

        M_left = cv2.moments(mask_left)
        M_left2 = cv2.moments(mask_left2)
        M_middle = cv2.moments(mask_middle)


        if M_middle['m00'] > 0:
            cx = int(M_middle['m10'] / M_middle['m00'])
            cy = int(M_middle['m01'] / M_middle['m00'])
            #d = rospy.Duration(10, 2)
            while M_middle['m00'] > 0 and self.counter == 0:
                rospy.sleep(rospy.Duration(3))
                self.counter = 1
                break
            self.twist.linear.x = 1.0
            self.cmd_vel_pub.publish(self.twist)
            rospy.loginfo("searchsssssssssssssssssssssssssssssssssssssssssssssssssssssssssss!")

        elif M_left['m00'] > 0:
            cx = int(M_left['m10'] / M_left['m00'])
            cy = int(M_left['m01'] / M_left['m00'])
            cv2.circle(image, (cx + 255, cy + 100), 20, (0, 0, 255), -1)
            err = cx * 3.5 - w / 2
            self.twist.linear.x = 1.0
            self.twist.angular.z = -float(err) / 300
            self.cmd_vel_pub.publish(self.twist)
            rospy.loginfo("yellow_line")

        elif M_left2['m00'] > 0:
            cx = int(M_left2['m10'] / M_left2['m00'])
            cy = int(M_left2['m01'] / M_left2['m00'])
            cv2.circle(image, (cx + 255, cy + 100), 20, (0, 0, 255), -1)
            err = cx * 3.5 - w / 2
            self.twist.linear.x = 1.0
            self.twist.angular.z = -float(err) / 300
            self.cmd_vel_pub.publish(self.twist)
            rospy.loginfo("white_line")

        mask_image = self.bridge.cv2_to_imgmsg(mask_middle)
        self.mask_pub.publish(mask_image)

        image_msg = self.bridge.cv2_to_imgmsg(image,'bgr8')
        self.p2_image_pub.publish(image_msg)
        rospy.loginfo('count = %d', self.count)
        self.count += 1


rospy.init_node('follower')
follower = Follower()
rospy.spin()

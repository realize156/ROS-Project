#!/usr/bin/env python
# -*- coding: utf-8 -*-
import cv2
import numpy
import cv_bridge
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from Mask import Mask


class line_follower:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)

        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.mask_pub = rospy.Publisher('camera/rgb/image_raw/mask', Image, queue_size=1)
        self.hsv_pub = rospy.Publisher('camera/rgb/image_raw/hsv', Image, queue_size=1)
        self.twist = Twist()
        self.count = 0
        self.counter = 0
        self.first_start_check = False
        self.mask = Mask()

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = numpy.array([25, 20, 50])
        upper_yellow = numpy.array([35, 255, 255])

        lower_white = numpy.array([0, 0, 0])
        upper_white = numpy.array([0, 0, 255])

        lower_red = numpy.array([0, 200, 0])
        upper_red = numpy.array([0, 255, 255])

        lower_white2 = numpy.array([0, 0, 190])
        upper_white2 = numpy.array([25, 36, 255])

        mask_yello = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        mask_blocking_bar = cv2.inRange(hsv, lower_red, upper_red)

        mask_middle = cv2.inRange(hsv, lower_white2, upper_white2)

        mask_right = cv2.inRange(hsv,lower_white,upper_white)

        h, w, d = image.shape
        search_top = 3 * h / 4
        search_bot = 3 * h / 4 + 20
        search_across = w / 2
        search_right = w * 2 / 3
        search_left = w / 3

        #################################


        mask_middle[0:h - 120, 0:w] = 0
        mask_middle[0:h, 0:300] = 0
        mask_middle[0:h, w - 200:w] = 0


        M_right = cv2.moments(mask_right)


        #M_left = self.mask.mask_callback(msg, 0,0,0,0,"yellow")
        M_left = cv2.moments(self.mask.mask_callback(msg,search_top,search_bot,0,search_across,"yellow"))
        M_left2 = cv2.moments(self.mask.mask_callback(msg,search_top,search_bot,0,search_across,"white"))

        M_middle = cv2.moments(mask_middle)
        ##################################

        mask_yello[0:360, 0:w] = 0
        mask_yello[380:h, 0:w] = 0
        M = cv2.moments(mask_yello)

        if self.first_start_check:
            if M_middle['m00'] > 0:
                cx = int(M_middle['m10'] / M_middle['m00'])
                cy = int(M_middle['m01'] / M_middle['m00'])
                # d = rospy.Duration(10, 2)
                while M_middle['m00'] > 0 and self.counter == 0:
                    rospy.sleep(rospy.Duration(3))
                    self.counter = 1
                    break
                self.twist.linear.x = 1.0
                self.cmd_vel_pub.publish(self.twist)
                rospy.loginfo("search stop line!")
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

        mask_blocking_bar[0:200, 0:w] = 0
        mask_blocking_bar[300:h, 0:w] = 0
        mask_blocking_bar[0:h, 0:200] = 0
        mask_blocking_bar[0:h, 400:w] = 0

        red_count = cv2.findContours(mask_blocking_bar, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)[1]  # 빨간 무늬 개수 구하기
        print(len(red_count))
        if len(red_count) == 0:
            self.first_start_check = True
        else:
            self.first_start_check = False

        #########################
        mask_image = self.bridge.cv2_to_imgmsg(mask_middle)
        self.mask_pub.publish(mask_image)
        mask_image = self.bridge.cv2_to_imgmsg(mask_blocking_bar)
        image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        hsv_image_msg = self.bridge.cv2_to_imgmsg(hsv)
        self.mask_pub.publish(mask_image)
        self.hsv_pub.publish(hsv_image_msg)



rospy.init_node('line_follower')
follower = line_follower()
rospy.spin()

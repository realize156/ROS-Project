#!/usr/bin/env python
# -*-coding: utf-8 -*-
#정지 표지판 인식 
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import rospy, cv2, cv_bridge, numpy

class Pyoji:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
        self.twist = Twist()

    def image_callback(self,msg):
        pic = cv2.imread('/home/shin/catkin_ws/src/deu_car/world/models/stop_sign_resize/materials/textures/StopSign_Diffuse.png')
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        pic_gray = cv2.cvtColor(pic,cv2.COLOR_BGR2GRAY)
        image_gray = cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
        sift = cv2.xfeatures2d.SIFT_create()
        kp1, des1 = sift.detectAndCompute(image_gray, None)
        kp2, des2 = sift.detectAndCompute(pic_gray, None)
        bf = cv2.BFMatcher()

        matches = bf.match(des1,des2)
        sorted_matches = sorted(matches, key=lambda x: x.distance)
        res = cv2.drawMatches(image, kp1, pic, kp2, sorted_matches[:30], None, flags=2)

        print(matches[0].distance)
        if matches[0].distance > 400:
       print("detect")
        else :
            self.twist.linear.x = 1.0
       print("not detect")
        self.cmd_vel_pub.publish(self.twist)

        cv2.imshow("res",res)
        cv2.waitKey(3)

rospy.init_node('test')
test = Test()
rospy.spin()

#!/usr/bin/env python
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist


class ObstacleDetection:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image,queue_size=1)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber('/camera/depth/image_raw',Image,self.callback,queue_size=1)
    self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
    self.twist = Twist()
  
  def doesImageContainObject(self,img):
    return (~np.isnan(img).any(axis=0).all())

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
    except CvBridgeError as e:
      print(e)

    cv_image_process = cv_image[:280,:]
    
    if(self.doesImageContainObject(cv_image_process) == True):
      print("I see something!")
      self.twist.linear.x = 0.0

    else:
      print("The coast is clear.")
      self.twist.linear.x = 1.0

    self.cmd_vel_pub.publish(self.twist)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "32FC1"))
    except CvBridgeError as e:
      print(e)

def main(args):
  od = ObstacleDetection()
  rospy.init_node('Obstacle_detection', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

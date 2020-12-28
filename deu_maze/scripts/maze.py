#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class maze:
    def __init__(self):
	
	self.scan_sub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
	self.odometry_sub = rospy.Subscriber('odom', Odometry, self.odometry_callback)
	self.cmd_vel_pub = rospy.Publisher('cmd_vel_mux/input/teleop', Twist, queue_size=1)
	# var
	self.g_range_ahead = 0
	self.g_range_left = 0
	self.g_range_right = 0

	self.departure_check = True # 출발지에서 90도 회전하면 False
	self.destination_check = False # 목적지에 도착하면 True
	self.start_check = False # 출발하기 전 90도 회전하면 True
	self.end_check = False # 목적지에 도착하고 다시 출발지에 도착하면 True 

        self.odom_x = 0
	self.odom_y = 0

	self.twist = Twist()
	rospy.init_node('maze')

	self.rate = rospy.Rate(10)

    def scan_callback(self, msg):
	self.g_range_ahead = msg.ranges[len(msg.ranges)/2]
	self.g_range_left = msg.ranges[len(msg.ranges)/2 + 200]
	self.g_range_right = msg.ranges[len(msg.ranges)/2 - 200]

    def odometry_callback(self, msg):
	self.odom_x = msg.pose.pose.position.x - 7.92
	self.odom_y = msg.pose.pose.position.y + 5.30

	# 도착지 좌표 확인
	if((self.odom_x > 7.75 and self.odom_x < 8.15) and (self.odom_y > -5.35 and self.odom_y < -4.70)):
		self.destination_check = True

	# 출발지 좌표 확인
	if ((self.odom_x > -8.12 and self.odom_x < -7.72) and (self.odom_y > 5.10 and self.odom_y < 5.75)):
		self.end_check = True
	else:
		self.end_check = False
	
    def straight(self):
	if(self.g_range_ahead < 1.0):
	  self.twist.linear.x = 0.2
	else:
	  self.twist.linear.x = 1.0
	#self.cmd_vel_pub.publish(self.twist)

    def left(self):
	self.twist.angular.z = 1.0 * (math.pi / 2.0)
	self.cmd_vel_pub.publish(self.twist)

    def right(self):
	self.twist.angular.z = -1.0 * (math.pi / 2.0)
	self.cmd_vel_pub.publish(self.twist)

    def stop(self):
	self.rate.sleep()

    def start(self):
	while not rospy.is_shutdown():
		if(self.departure_check is True): # 처음에 출발하기 전 오른쪽으로 90도 회전
			for i in range(8):
				self.right()
				self.rate.sleep()
			self.departure_check = False
			self.start_check = True	

		elif(self.destination_check is True): # 목적지 좌표에 도착하면 다시 뒤로 회전
			self.start_check = False
			for i in range(8):
				self.right()
				self.rate.sleep()
			self.destination_check = False

		elif(self.start_check is True): # 좌수법
			if(self.g_range_ahead < 1.0 or self.g_range_left < 1.2):
				
				self.straight()
				self.right()
				#self.stop()
			else:
				self.straight()
				self.left()
				#self.stop()

		else: # 우수법
			if(self.g_range_ahead < 1.0 or self.g_range_right < 1.2):
				
				self.straight()
				self.left()
				#self.stop()
			else:
				self.straight()
				self.right()
				#self.stop()

if __name__ == "__main__":
    maze = maze()
    maze.start()


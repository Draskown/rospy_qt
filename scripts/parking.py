#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Needed imports
import rospy
from time import sleep
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class Parking():
	def __init__(self):
		# Initializing of the global values
		self.distance = 0
		self.parking = False
		self.plan = True

		# Set subscibers for ROS topics
		sub_sign = rospy.Subscriber('sign', String, self.cb_sign, queue_size=1)
		sub_scan = rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)

		# Set a publisher for robot's velocity, moving along the line flag and to public log messages
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
		self.pub_msg = rospy.Publisher('log_msg', String, queue_size=1)

		# While ROS is not shut down and if parking flag is true
		# Perform parking
		while not rospy.is_shutdown():
			try:
				if self.plan:
					if(self.parking == True):
						self.pub_msg.publish("Start parking mission\r\n")
						rospy.sleep(6.3)
						if(self.distance > 1 or self.distance == 0):
							self.do_parking()
						else:
							self.pub_msg.publish("Place is occupied\r\n")
							rospy.sleep(1.5)
							self.do_parking()
						break
				else:
						rospy.signal_shutdown('force ending')
						break
			except KeyboardInterrupt:
				break

	# Listens for the plan's state
	def cbPlan(self, data):
		self.plan = data.data

	# Listens for the sign's data
	def cb_sign(self, data):
		if(data.data == "parking"):
			self.parking = True

	# Listens for the lidar's data
	def cb_scan(self, data):
		self.counter = 0
		for i in range(200, 300):
			if(data.ranges[i] < 1):
				self.distance += data.ranges[i]
				self.counter += 1
		if(self.counter != 0):
			self.distance /= self.counter
		else:
			self.distance = 0

	# Sets the robot's velocity
	def pub_velocity(self, x, z, time):
		vel = Twist()
		for _ in range(0, int(time*10)):
			vel.linear.x = x
			vel.angular.z = z
			self.pub_vel.publish(vel)
			rospy.sleep(0.1)

	# Do the handling of parking
	def do_parking(self):
		# Disable moving along the lines
		flag_move_line = Bool()
		flag_move_line.data = False
		rospy.sleep(0.1)
		self.pub_line_move.publish(flag_move_line)    
		
		# Stop the robot
		for i in range(20,0, -2):
			self.pub_velocity(i/100,0,0.1)
		
		# Turn the robot
		self.pub_velocity(0, -0.4,4.3)
		
		# Move the robot further
		self.pub_velocity(0.13, 0, 1.8)
		
		# Stop the robot
		for i in range(20,0, -2):
			self.pub_velocity(i/100,0,0.1)
			
		# Turn the robot back
		self.pub_velocity(-0.13, 0, 1.8)
		
		# Stop the robot
		for i in range(20,0, -2):
			self.pub_velocity(-i/100,0,0.1)
		
		# Enable moving along the line
		self.pub_velocity(0, 0.4,4)
		self.pub_velocity(0,0,0.5)
		flag_move_line.data = True
		self.pub_line_move.publish(flag_move_line)
		self.pub_msg.publish("Finish parking mission\r\n")


if __name__ == '__main__':
	rospy.init_node('parking')
	try:
		node = Parking()
	except rospy.ROSInterruptException:
		pass
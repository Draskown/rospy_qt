#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from std_msgs.msg import  String, Bool
from geometry_msgs.msg import Twist


class LightController():
	def __init__(self):
		# Initializing of global values
		self.light = False
		self.enabled = False
		self.plan = True
		
		# Set subscribers for ROS topics
		sub_sign = rospy.Subscriber('traffic_light', String, self.cb_traffic_light, queue_size=1)
		sub_state = rospy.Subscriber('state', String, self.cb_ts, queue_size=1)

		# Set publishers for robot's velocity and a flag to whether use line moving or not
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)

		# Wait while the light is detected
		# Then do the handling
		while not rospy.is_shutdown():
			try:
				if self.plan:
					if self.light:
						print("start traffic light mission")
						self.do_traffic_light()
						break
					else:
						rospy.sleep(0.1)
				else:
					break
			except KeyboardInterrupt:
				break

	# Listens for the plan's state
	def cbPlan(self, data):
		self.plan = data.data

	# Listens for the robot's state
	def cb_ts(self, data):		
		if int(data.data) == 1 or data.data == "2":
			self.enabled = True
			
	# Listents for the traffic light's detection state
	def cb_traffic_light(self, data):
		if self.enabled:
			if(data.data == "yellow" or data.data == "red"):
				self.light = True
			elif(data.data == "green"):
				self.light = False

	# Sets the velocity for the robot
	def pub_velocity(self, x, z, time):
		vel = Twist()
		for _ in range(0, int(time*10)):
			vel.linear.x = x
			vel.angular.z = z
			self.pub_vel.publish(vel)
			rospy.sleep(0.1)

	# Handling of the traffic light mission
	def do_traffic_light(self):
		# Remove the flag of moving along the lines
		flag_move_line = Bool()
		flag_move_line.data = False
		rospy.sleep(0.1)

		# Publiish that robot does not follow the lines anymore
		self.pub_line_move.publish(flag_move_line)
		
		# Stop the robot
		for i in range(5,0, -1):
			self.pub_velocity(i/100,0,0.1)
		
		print("published stop msg")

		# Do not move the robot until the light is green
		while(self.light == True):
			self.pub_velocity(0, -0.05, 0.1)

		# Set the flag to move along the lines back
		flag_move_line.data = True
		self.pub_line_move.publish(flag_move_line)


if __name__ == '__main__':
	rospy.init_node('light_controller', disable_signals=True)
	try:
		node = LightController()
	except rospy.ROSInterruptException:
		pass
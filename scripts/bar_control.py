#!/usr/bin/env python3
# -*- coding: utf-8 -*-


# needed imports
import rospy
from time import sleep
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class BarControl():
	def __init__(self):
		# initializing of global values
		self.stop_bar = False
		self.crunches = False
		self.orientation = 0
		self.pose_x = 0
		self.pose_y = 0
		self.error = 0
		self.target_orientation = 0.7
		self.target_position = {'x': -1.828144623628639, 'y': 1.352652814982841}
		self.msg = String()

		self.plan = True
		self.state = ""

		# Susbcribers for ROS topics 
		sub_bar = rospy.Subscriber('bar', String, self.cb_bar, queue_size=1)
		sub_odom = rospy.Subscriber('odom', Odometry, self.cb_odom, queue_size=1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)
		sub_ts = rospy.Subscriber('state', String, self.cb_ts, queue_size=1)

		# Publisher for ROS topics
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pub_msg = rospy.Publisher('log_msg', String, queue_size=1)

		# Move endlessly until flag 'crunches' is True
		while not rospy.is_shutdown():
			try:
				if self.plan:
					if self.crunches:
						self.do_stop()
						break
				else:
					rospy.signal_shutdown('force edning')
					break
			except KeyboardInterrupt:
				break

	
	# Listens for the state of a vehicle in the mission
	def cb_ts(self, data):
		self.state = data.data

	# Listens whether the plan has changed
	def cbPlan(self, data):
		self.plan = data.data

	# Listens for odomentry information
	def cb_odom(self, msg):
		self.pose_x = msg.pose.pose.position.x
		self.pose_y = msg.pose.pose.position.y
		self.orientation = msg.pose.pose.orientation.z

		if self.pose_x > -1.92 and self.pose_x < -1.72 and self.pose_y > 0 and self.pose_y < 1.7:
			self.crunches = True

	# Listens whether the bar has been detected or not
	def cb_bar(self, data):
		if data.data == "none":
			self.stop_bar = False
		else:
			self.stop_bar = True

	# Moves the robot
	def pubvel(self, x, z, time):
		velocity = Twist()
		velocity.linear.x = x
		velocity.angular.z = z
		self.pub_vel.publish(velocity)
		rospy.sleep(time)	

	def do_stop(self):		
		# Stop for a limited time
		rospy.sleep(1.45)	

		# Disable moving along the lines by publishing the false state
		pub_line_move = rospy.Publisher('line_move_flag', Bool, queue_size=1)
		flag_move_line = Bool()
		flag_move_line.data = False
		rospy.sleep(0.1)
		pub_line_move.publish(flag_move_line)
		
		self.msg.data = 'Moving forward \r\n'
		self.pub_msg.publish(self.msg)
		
		# If robot has not yet arrived at desired location
		while not (abs(self.pose_x + 1.75) < 0.15 and self.pose_y < 1):
			self.error = self.target_orientation - abs(self.orientation)
			self.pubvel(0.2, -self.error*5, 0.1)
		
		# Move it forward
		for i in range(20,0, -2):
			self.pubvel(i/100,0,0.2)
		
		# Stop the robot
		self.pubvel(0.0, 0.0, 0.1)
		
		self.pub_msg.publish('Waiting for the bar to open \r\n')
		# Wait for the bar to open
		while self.stop_bar:
			if self.state == "8":
				break
			self.pubvel(0.0, 0.0, 0.1)
		
		# Correct the position of the robot
		while self.pose_y > 0:
			self.error = self.target_orientation - abs(self.orientation)
			self.pubvel(0.2, -self.error*5, 0.1)
	

if __name__ == '__main__':
	rospy.init_node('bar_control')
	try:
		node = BarControl()
	except rospy.ROSInterruptException:
		pass

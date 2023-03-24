#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Needed imports
import rospy
from time import sleep
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist

class LineControl():
	def __init__(self):
		# Initializng of global values
		self.integral = 0
		self.move_flag = True
		self.plan = True

		# Set a publisher for the robot's velocity
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)

		# Set subscribers for the ROS topics
		sub_image = rospy.Subscriber('line_error', Float64, self.cbError, queue_size=1)
		sub_move_flag = rospy.Subscriber('line_move_flag', Bool, self.cb_flag, queue_size=1)
		sub_ts = rospy.Subscriber('state', String, self.cb_ts, queue_size=1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)

		# Listen to every subscriber of topics
		# Until the package is shut down
		while not rospy.is_shutdown():
			try:
				if self.plan:
					rospy.sleep(0.1)
				else:
					rospy.signal_shutdown('force ending')
					break
			except KeyboardInterrupt:
				break

	# Listent for the plan's state
	def cbPlan(self, data):
		self.plan = data.data

	# Listents for the robot's state
	def cb_ts(self, data):
		if int(data.data) >= 5:
			self.move_flag = False

	# Listens for the error in robot's position messages
	# And corrects the position of the robot by the error
	def cbError(self, error):
		if self.move_flag:
			velocity = Twist()
			self.integral += + 0.000005*error.data
			proportional = 0.01*error.data
			up = proportional+self.integral
			velocity.angular.z = up
			velocity.linear.x = 0.22 - 0.09*abs(up)
			
			self.pub_vel.publish(velocity)

	# Listens for the flag whether moving along the line is enabled
	def cb_flag(self, data):
		self.move_flag = data.data


if __name__ == '__main__':
	rospy.init_node('line_control', disable_signals=True)
	try:
		node = LineControl()
	except rospy.ROSInterruptException:
		pass

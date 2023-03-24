#!/usr/bin/env python3
# -*- coding: utf-8 -*-


'''
Coordinates of important places on the map

Traffic light: {0, 0.3}, 0.2
Parking: {1.60, 2.00}, 1.83
Bar: {2.95, 3.00}, 3
Tunnel: 3.07
'''

# Needed imports
import rospy
from std_msgs.msg import Float64, Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class OdomCalculator():
	def __init__(self):
		# Initializing for the global values
		self.x = Float64()
		self.x.data = 0.0
		self.plan = True
		self.ros_plan = Bool()
		self.msg = ""
		self.log_msg = String()
		self.closest_ten = 2
		self.k = 0

		# Set publishers for the custom odometry calculation, plan's state
		# And to publish the log messages
		self.odom_pub = rospy.Publisher('codom', Float64, queue_size=1)
		self.plan_pub = rospy.Publisher('plan', Bool, queue_size=1)
		self.pub_msg = rospy.Publisher('log_msg', String, queue_size=1)

		# Set subscribers for the ROS topics
		cmd_sub = rospy.Subscriber('cmd_vel', Twist, self.cb_vel, queue_size=1)
		scan_sub = rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)
		sign_msg_sub = rospy.Subscriber('sign', String, self.cb_sign, queue_size=1)
		tl_msg_sub = rospy.Subscriber('tl_msg', String, self.cb_tl, queue_size=1)
		bar_msg_sub = rospy.Subscriber('bar', String, self.cb_bar, queue_size=1)

		# Handle the class until the ROS is shut down
		while not rospy.is_shutdown():
			self.main()

	# Publishing of the plan's state
	def main(self):
		if self.plan:
			rospy.sleep(0.1)
			self.pubplan()
		else:
			print(self.msg)
			self.log_msg.data = self.msg + "\r\n"
			self.pub_msg.publish(self.log_msg)
			self.pubplan()
			rospy.signal_shutdown('force ending')
			return

	# Listens for the robot's velocity
	def cb_vel(self, velocity):
		self.x.data += velocity.linear.x / 100.0

		# And then corrects the robot's position
		self.odom_pub.publish(self.x)

	# Listens for the lidar's data
	def cb_scan(self, data):
		self.k = 0

		# Count the points that are close to the robot
		if self.x.data <= 0.1:
			for i in range(359):
				if i < 235 and i > 65:
					continue
				if data.ranges[i] < self.closest_ten:
					self.k+=1

			# If there are many - print that the robot is not in the desired place
			if self.k > 3:
				self.plan = False
				self.msg = "I'm in the wrong place"

	# Listens for the traffic light's state
	def cb_tl(self, data):
		# If the robot is not in the supposed place - print the error
		if (data.data != "none" and self.x.data > 0.3) or (self.x.data > 0.2 and self.x.data <= 0.22 and data.data == "none"):
			self.plan = False
			self.msg = "Traffic light is not here"
		# If the robot is - correct the position of the robot
		if data.data != "none" and self.x.data <= 0.3:
			self.x.data = 0.2

	# Listens for the parking sign's state
	def cb_sign(self, data):
		# If the robot is not in the supposed place - print the error
		if (data.data == "parking" and (self.x.data > 2 or self.x.data < 1.6) and self.x.data > 0.3) or (self.x.data > 1.82 and self.x.data < 1.83 and data.data == "none"):
			self.plan = False
			self.msg = "Parking is not here"
		# If the robot is - correct the position of the robot
		if data.data != "none" and self.x.data <= 1.95 and self.x.data > 1.6:
			self.x.data = 1.83

	# Listens for the bar's state
	def cb_bar(self, data):
		# If the robot is not in the supposed place - print the error
		if (data.data != "none" and (self.x.data > 3.05 or self.x.data < 2.87) and self.x.data > 2.00) or (self.x.data > 2.99 and self.x.data < 3.0 and data.data == "none"):
			self.plan = False
			self.msg = "Bar is not here"
		# If the robot is - correct the position of the robot
		if data.data != "none" and self.x.data <= 3.01 and self.x.data > 2.9:
			self.x.data = 3.00
	
	# Publishes the plan's state
	def pubplan(self):
		self.ros_plan.data = self.plan
		self.plan_pub.publish(self.ros_plan)		


if __name__ == "__main__":
	rospy.init_node('odometry', disable_signals = True)
	try:
		bardetect = OdomCalculator()
	except rospy.ROSInterruptException:
		pass

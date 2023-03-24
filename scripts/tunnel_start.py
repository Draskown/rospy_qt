#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Needed imports
import rospy
from time import sleep
from std_msgs.msg import Bool, String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# S
def sign(data):
	if(data >= 0):
		return 1
	else:
		return -1


class TunnelStart():
	def __init__(self):
		# Initializing of global values
		self.ranges = list()
		self.tunnel = False
		self.in_tunnel = False
		self.end_of_mission = False
		self.plan = True
		self.state = "1"
		self.bar = True
		self.log_msg = String()

		# Set a publisher for the robot's velocity
		self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.pub_msg = rospy.Publisher('log_msg', String, queue_size=1)

		# Set subscribers for the ROS topics
		sub_scan = rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)
		sub_ts = rospy.Subscriber('state', String, self.cb_ts, queue_size=1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)
		sub_sign = rospy.Subscriber('sign', String, self.cb_sign, queue_size = 1)
		sub_bar = rospy.Subscriber('bar', String, self.cb_bar, queue_size=1)
		
		# While ROS is not shut down - listen for the tunnel detection
		while not rospy.is_shutdown():
			try:
				if not self.end_of_mission and self.plan:				
					if(self.tunnel and not self.in_tunnel and self.bar == "none"):
						self.log_msg.data = "Tunnel detected\r\n"
						self.pub_msg.publish(self.log_msg)
						rospy.sleep(3)
						self.in_tunnel = True
					elif self.in_tunnel:
						self.in_tunnel_go()
				else:
					rospy.signal_shutdown('force ending')
					break
			except KeyboardInterrupt:
				break

	# Listens for the plan's state
	def cbPlan(self, data):
		self.plan = data.data

	# Listens for the bar's state
	def cb_bar(self, data):
		self.bar = data.data

	# Listens for the robot's state
	def cb_ts(self, data):
		self.state = data.data

	# Listens for the sign
	def cb_sign(self, data):
		if data.data == "tunnel":
			self.tunnel = True

	# Listens for the lidar's data
	def cb_scan(self, data):
		self.ranges = data.ranges

	# Sets the robot's velocity
	def pub_velocity(self, x, z, time):
		vel = Twist()
		for _ in range(0, int(time*10)):
			vel.linear.x = x
			vel.angular.z = z
			self.pub_vel.publish(vel)
			rospy.sleep(0.1)

	# Handle the moving in the tunnel
	def in_tunnel_go(self):
		if len(self.ranges) == 0:
			return

		# If the ray facing forward returns a big value	
		if self.ranges[300] > 0.32:
			# Set end of mission flat
			self.end_of_mission = True

			# Move the robot forward
			self.pub_velocity(0.15, 0, 1)

			# Stop the robot gradually
			for i in range(20,0, -2):
				self.pub_velocity(i/100,0,0.3)
			
			# Stop the robot completely
			self.pub_velocity(0,0,0.1)
			return
		
		# Set constant velocities
		vel_x = 0.12
		vel_z = 0
		error = 4*(0.18 - self.ranges[300])

		# Handle the error for the sign's detection
		if(abs(error) > 1.5):
			error = sign(error)*1.5
		vel_z = error

		# Move along the right wall
		for i in range(0,360,1):
			if(self.ranges[i] < 0.15):
				if(i <= 30 or i >= 330):
					vel_x = -0.09
					vel_z = 0.4    
		self.pub_velocity(vel_x, vel_z, 0.1)


if __name__ == '__main__':
	rospy.init_node('tunnel_start')
	try:
		tunnelstart = TunnelStart()
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python3


import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import sys, select, os
if os.name == 'nt':
	import msvcrt
else:
	import tty, termios


def makeSimpleProfile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output


def constrain(input, low, high):
	if input < low:
		input = low
	elif input > high:
		input = high
	else:
		input = input

	return input


class Teleop():
	def __init__(self):
		self.max_lin_vel = 0.5
		self.max_ang_vel = 5
		self.lin_vel_step = 0.005
		self.ang_vel_step = 0.5

		self.e = """
		Communications Failed
		"""

		self.key = " "

		self.target_linear_vel   = 0.0
		self.target_angular_vel  = 0.0
		self.control_linear_vel  = 0.0
		self.control_angular_vel = 0.0

		self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
		self.pub_msg = rospy.Publisher('log_msg', String, queue_size=1)

		btn_sub = rospy.Subscriber('teleop_btn', String, self.cb_btn, queue_size=1)

		self.main()

	def cb_btn(self, data):
		self.key = data.data

	def checkLinearLimitVelocity(self, vel):
			return constrain(vel, -self.max_lin_vel, self.max_lin_vel)

	def checkAngularLimitVelocity(self, vel):
			return constrain(vel, -self.max_ang_vel, self.max_ang_vel)
	
	def main(self):
		try:
			while not rospy.is_shutdown():
				target_angular_vel = 0.0
				
				if self.key == 'w' :
					if target_linear_vel >= 0.0:
						target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel + self.lin_vel_step)
					else:
						target_linear_vel = 0.0
				
				elif self.key == 's':
					if target_linear_vel <= 0.0:
						target_linear_vel = self.checkLinearLimitVelocity(target_linear_vel - self.lin_vel_step)
					else:
						target_linear_vel = 0.0
				
				elif self.key == 'a':
					if target_angular_vel >= 0.0:
						target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel + self.ang_vel_step)
					else:
						target_angular_vel = 0.0
				
				elif self.key == 'd':
					if target_angular_vel <= 0.0:
						target_angular_vel = self.checkAngularLimitVelocity(target_angular_vel - self.ang_vel_step)
					else:
						target_angular_vel = 0.0
				
				elif self.key == 's_r':
					target_angular_vel  = 0.0
					control_angular_vel = 0.0

				elif self.key == ' ':
					target_linear_vel   = 0.0
					control_linear_vel  = 0.0
					target_angular_vel  = 0.0
					control_angular_vel = 0.0

				twist = Twist()

				control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (self.lin_vel_step/2.0))
				twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

				control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (self.ang_vel_step/2.0))
				twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

				self.pub.publish(twist)

		except:
			self.pub_msg.publish(self.e)

		finally:
			twist = Twist()
			twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
			self.pub.publish(twist)


if __name__=="__main__":
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('turtlebot3_teleop')

	try:
		node = Teleop()
	except rospy.ROSInterruptException:
		pass

	if os.name != 'nt':
		termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

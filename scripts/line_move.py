#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# OBSOLETE CODE!! NOT RELEVANT!!
#
#
#
#
#
#
#


import rospy, sys, select, os, tty, termios
from time import sleep
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import Twist
integral = 0
orientation = 0
error = 0
move_flag = False
crunches = False
key = ' '

pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
velocity = Twist()


crunch_position = {'x': -1.828144623628639, 'y': 1.352652814982841}
crunch_orientation = 0.698


def pubvel(x, z):
	velocity.linear.x = x
	velocity.angular.z = z
	
	if crunches:
		print(velocity.angular.z)
	
	pub_vel.publish(velocity)


def cb_odom(msg):
	global crunches, move_flag, orientation, error, integral, proportional
	pose_x = msg.pose.pose.position.x
	pose_y = msg.pose.pose.position.y
	orientation = msg.pose.pose.orientation.z
	
	if abs(pose_x - crunch_position['x']) <= 0.12 and abs(pose_y - crunch_position['y']) <= 0.16 and abs(abs(orientation) - crunch_orientation) <= 0.01 and not crunches:
		move_flag = False
		crunches = True
	
	if crunches and pose_y > 0:
		error = crunch_orientation - abs(orientation)
		pubvel(0.22, -error)
		rospy.sleep(0.1)
	elif crunches and pose_y < 0:
		print('Done moving')
		pubvel(0.0, 0.0)
		rospy.signal_shutdown('')


def getKey():
	if os.name == 'nt':
		if sys.version_info[0] >= 3:
			return msvcrt.getch().decode()
		else:
			return msvcrt.getch()

	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def cbError(error):
	global integral, move_flag
	key = getKey()
	
	if key == 'r':
		move_flag = True
		print('The moving has been started\n')
	elif key == 'q':
		print('The moving has been stopped\n')
		move_flag = False
		pubvel(0.0, 0.0)
	elif key == '\x03':
		rospy.signal_shutdown('')
		exit(0)
		
	if(move_flag == True):
		integral = integral + 0.000005*error.data
		proportional = 0.01*error.data
		up = proportional+integral
		pubvel(0.22 - 0.09*abs(up), up)


def cb_flag(data):
	global move_flag
	move_flag = data.data


if __name__ == '__main__':
	print('\nPress R to start and Q to stop\n')
	
	if os.name != 'nt':
		settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('line_move')
	
	sub_image = rospy.Subscriber('line_error', Float64, cbError, queue_size=1)
	sub_move_flag = rospy.Subscriber('line_move_flag', Bool, cb_flag, queue_size=1)
	sub_odom = rospy.Subscriber('odom', Odometry, cb_odom, queue_size = 1)
	while not rospy.is_shutdown():
		try:
			rospy.sleep(0.1)
		except KeyboardInterrupt:
			break


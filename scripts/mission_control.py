#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
Coordinates of important points on the map:

Traffic light: {0, 0.3}, 0.2
Parking: {1.60, 1.90}, 1.83
Bar: {2.95, 3.00}, 2.95
Tunnel: 3.16
'''

# Needed imports
import rospy
import time
import numpy as np
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Pose
from std_srvs.srv import Empty
from gazebo_msgs.srv import SpawnModel, DeleteModel


class ControlMission():
	def __init__(self):
		self.rate = rospy.Rate(5)
		self.initial_pose = Pose()
		self.traffic_state = 1
		self.path = 0.0
		
		self.plan = True

		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
		sub_odom = rospy.Subscriber('codom', Float64, self.getOdom, queue_size = 1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size=1)
		
		self.pub_state = rospy.Publisher('state', String, queue_size=1)
		
		self.loadMissionModel()
		self.setTraffic()
		self.controlMission()
	
	def cbPlan(self, data):
		self.plan = data.data
	
	def getOdom(self, msg):
		self.path = msg.data
		
		if self.path > 2.88 and self.traffic_state == 4:
			self.traffic_state = 5
			
		if abs(self.path - 3.0) < 0.05 and self.traffic_state == 6:
			self.traffic_state = 7
			self.current_time = time.time()
	
	def loadMissionModel(self):
		model_dir_path = '/root/ros_workspace/src/turtlebot3_autorace/turtlebot3_autorace_core/nodes'
		model_dir_path = model_dir_path.replace('/turtlebot3_autorace/turtlebot3_autorace_core/nodes','/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_autorace')

		turtlebot_dir_path = '/root/ros_workspace/src/turtlebot3_autorace/turtlebot3_autorace_core/nodes'
		turtlebot_dir_path = turtlebot_dir_path.replace('/turtlebot3_autorace/turtlebot3_autorace_core/nodes', '/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf')

		red_light_path = model_dir_path + '/traffic_light_red/model.sdf'
		red_light_model = open(red_light_path,'r')
		self.red_light_model = red_light_model.read()

		yellow_light_path = model_dir_path + '/traffic_light_yellow/model.sdf'
		yellow_light_model = open(yellow_light_path, 'r')
		self.yellow_light_model = yellow_light_model.read()

		green_light_path = model_dir_path + '/traffic_light_green/model.sdf'
		green_light_model = open(green_light_path, 'r')
		self.green_light_model = green_light_model.read()

		up_bar_path = model_dir_path + '/traffic_bar_up/model.sdf'
		up_bar_model = open(up_bar_path, 'r')
		self.up_bar_model = up_bar_model.read()

		down_bar_path = model_dir_path + '/traffic_bar_down/model.sdf'
		down_bar_model = open(down_bar_path, 'r')
		self.down_bar_model = down_bar_model.read()

		parking_model = open(turtlebot_dir_path, 'r')
		self.parking_model = parking_model.read().replace('<static>0', '<static>1')

	def setTraffic(self):
		rospy.wait_for_service('gazebo/spawn_sdf_model')
		spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
		spawn_model_prox('traffic_light_green', self.green_light_model, "robotos_name_space", self.initial_pose, "world")
		spawn_model_prox('up_bar', self.up_bar_model, "robotos_name_space", self.initial_pose, "world")
		parking_pose = Pose()
		parking_stop = np.random.rand()
		parking_pose.position.x = 1.14 if parking_stop < 0.5 else 0.69
		parking_pose.position.y = 1.50
		parking_pose.position.z = 0.03
		parking_pose.orientation.x = 0
		parking_pose.orientation.y = 0
		parking_pose.orientation.z = -1
		parking_pose.orientation.w = 1
		spawn_model_prox('praking_turtlebot3', self.parking_model, "robotos_name_space", parking_pose, "world")

	def controlMission(self):
		while not rospy.is_shutdown():
				if not self.plan:
					rospy.signal_shutdown('force ending')
					break
				
				if self.traffic_state == 1:  # turn on yellow light
					rospy.wait_for_service('gazebo/spawn_sdf_model')
					spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
					spawn_model_prox('traffic_light_yellow', self.yellow_light_model, "robotos_name_space", self.initial_pose, "world")
					del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
					del_model_prox('traffic_light_green')
					self.traffic_state = 2
					self.current_time = time.time()

				elif self.traffic_state == 2:
					if abs(self.current_time - time.time()) > 2:  # turn on red light after 3s.
						rospy.wait_for_service('gazebo/spawn_sdf_model')
						spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
						spawn_model_prox('traffic_light_red', self.red_light_model, "robotos_name_space",
                                     self.initial_pose, "world")
						del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
						del_model_prox('traffic_light_yellow')
						self.traffic_state = 3
						self.current_time = time.time()

				elif self.traffic_state == 3:
					if abs(self.current_time - time.time()) > 5:  # turn on green light after 5s.
						rospy.wait_for_service('gazebo/spawn_sdf_model')
						spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
						spawn_model_prox('traffic_light_green', self.green_light_model, "robotos_name_space",
												   self.initial_pose, "world")
						del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
						del_model_prox('traffic_light_red')
						self.traffic_state = 4

				elif self.traffic_state == 5:  # bar down.
					rospy.wait_for_service('gazebo/spawn_sdf_model')
					spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
					spawn_model_prox('down_bar', self.down_bar_model, "robotos_name_space",
					self.initial_pose, "world")
					del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
					del_model_prox('up_bar')
					self.traffic_state = 6

				elif self.traffic_state == 7:  # bar up
					if abs(self.current_time - time.time()) > 5:
						rospy.wait_for_service('gazebo/spawn_sdf_model')
						spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
						spawn_model_prox('up_bar', self.up_bar_model, "robotos_name_space",
						self.initial_pose, "world")
						del_model_prox = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
						del_model_prox('down_bar')
						self.traffic_state = 8
				
				self.pub_state.publish(str(self.traffic_state))
				self.rate.sleep()


if __name__ == '__main__':
	rospy.init_node('mission_control')
	try:
		controlmission = ControlMission()
	except rospy.ROSInterruptException:
		pass

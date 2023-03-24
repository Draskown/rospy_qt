#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Needed imports
import rospy, cv2, os, select, sys, tty, termios
import cv2, numpy as np
from std_msgs.msg import String, Bool, Int8
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge


class Xlidarcontrol():
	def __init__(self):
		if os.name != 'nt':
			self.settings = termios.tcgetattr(sys.stdin)

		# Initializing of global values
		self.plan = True
		self.front_mode = 1
		self.angle = 0
		self.closest = 0.75
		self.cvBridge = CvBridge()
		self.sign_text = "none"
		self.bar_text = "none"
		self.tl_text = "none"
		self.show_env = False
		
		self.empty_image = np.zeros(shape=(400, 400), dtype=np.uint8)

		# Set subscribers for ROS topics
		sign_msg_sub = rospy.Subscriber('sign', String, self.cb_sign, queue_size=1)
		sign_img_sub = rospy.Subscriber('sign_image', Image, self.cb_sign_img, queue_size=1)
		tl_msg_sub = rospy.Subscriber('tl_msg', String, self.cb_tl, queue_size=1)
		tl_sub = rospy.Subscriber('image_traffic_light', Image, self.cb_tl_img, queue_size = 1)
		bar_sub = rospy.Subscriber('image_bar', Image, self.cb_bar_img, queue_size=1)
		bar_msg_sub = rospy.Subscriber('bar', String, self.cb_bar, queue_size=1)
		camera_sub = rospy.Subscriber('camera/image', Image, self.cb_cam, queue_size=1)
		scan_sub = rospy.Subscriber('scan', LaserScan, self.cb_scan, queue_size=1)
		plan_sub = rospy.Subscriber('plan', Bool, self.cb_plan, queue_size=1)
		front_cam_sub = rospy.Subscriber('front_camera_mode', Int8, self.cb_front_mode, queue_size=1)
		show_env_sub = rospy.Subscriber('show_env', Bool, self.cb_switch_env, queue_size=1)
		
		# Set publishers for the image and lidar's data
		self.img_pub = rospy.Publisher('decided_img', Image, queue_size=1)
		self.scan_pub = rospy.Publisher('scan_img', Image, queue_size=1)
		
		# Handle the class until the ROS is shut down
		while not rospy.is_shutdown():
			self.main()

	# Listens for the plan's state
	def cb_plan(self, plan):
		self.plan = plan.data
		
	# Listens fot the sign
	def cb_sign(self, sign_name):
		self.sign_text = sign_name.data

	# Listens for the traffic light's state
	def cb_tl(self, data):
		self.tl_text = data.data
		
	# Listens for the bar's state
	def cb_bar(self, data):
		self.bar_text = data.data
	
	# Listens for the image
	def cb_cam(self, image):
		try:
			if self.front_mode == 1:
				self.pub_image(image, "rgb8")
		except AttributeError:
			return

	# Listens for the traffic light image
	def cb_tl_img(self, image):
		try:
			if self.front_mode == 2:
				self.pub_image(image, "rgb8")
		except AttributeError:
			return
		
	# Listens for the bar image
	def cb_bar_img(self, image):
		try:
			if self.front_mode == 3:
				self.pub_image(image, "rgb8")
		except AttributeError:
			return

	# Listens for the signs image
	def cb_sign_img(self, image):
		try:
			if self.front_mode == 4:
				self.pub_image(image, "rgb8")
		except AttributeError:
			return
	
	# Listens for the lidar's data
	def cb_scan(self, data):		
		closest_ten = 2
		self.closest = 0.75

		# Handle all the rays of lidar
		for i in range (359):
			# Calculate the angle of the robot in two cases
			if self.sign_text != "tunnel" and data.ranges[i] < self.closest and (i > 270 or i < 90):
				self.closest = data.ranges[i]
				if i > 180:
					self.angle = i-360
				else:
					self.angle = i+1
			elif self.sign_text == "tunnel" and data.ranges[i] < self.closest and (i > 350 or i < 90):
				self.closest = data.ranges[i]
				if i > 180:
					self.angle = i-360
				else:
					self.angle = i+1
			
			# Draw every fifth ray
			if i%5 != 0:
				if data.ranges[i] < closest_ten:
					closest_ten = data.ranges[i]
			else:
				if closest_ten != 2:
					x = int(closest_ten*np.cos((i + 180)*np.pi/180)*100 + 200)
					y = int(closest_ten*np.sin((i + 180)*np.pi/180)*100 + 200)
				
					self.empty_image[x, y] = 255
					closest_ten = 2
		
		# Publish the lidar's image and update the empty image
		cv_image_rgb = cv2.cvtColor(self.empty_image, cv2.COLOR_GRAY2RGB)
		self.scan_pub.publish(self.cvBridge.cv2_to_imgmsg(cv_image_rgb, "rgb8"))
		self.empty_image = np.zeros(shape=(400, 400), dtype=np.uint8)
	
	# Publish the image in selected mode
	def pub_image(self, img, encoding):		
		if self.show_env:
			seen_object = "none"
			if self.sign_text != "none" and self.bar_text == "none" and self.tl_text == "none":
				seen_object = self.sign_text
			elif self.sign_text == "none" and self.bar_text != "none" and self.tl_text == "none":
				seen_object = self.bar_text
			elif self.sign_text == "none" and self.bar_text == "none" and self.tl_text != "none":
				seen_object = self.tl_text
			elif self.sign_text != "none" and self.bar_text != "none" and self.tl_text == "none":
				seen_object = self.bar_text
			elif self.sign_text != "none" and self.bar_text == "none" and self.tl_text != "none":
				seen_object = self.tl_text
			elif self.sign_text == "none" and self.bar_text != "none" and self.tl_text == "none":
				seen_object = self.bar_text
			elif self.sign_text != "none" and self.bar_text != "none" and self.tl_text != "none":
				seen_object = self.bar_text
				
			# Put text to the image
			cv_image = self.cvBridge.imgmsg_to_cv2(img)
			cv_image = cv2.putText(cv_image, 
									seen_object,
									(3, 18),
									cv2.FONT_HERSHEY_SIMPLEX,
									0.75,
									(255, 0, 155),
									1,
									2)
			
			# Put the distance to the sign
			if seen_object != "none" and seen_object != "traffic light":	
				text = "distance: {} cm".format(int(self.closest*125))
				
				cv_image = cv2.putText(cv_image, 
										text,
										(3, 34),
										cv2.FONT_HERSHEY_SIMPLEX,
										0.4,
										(255, 0, 155),
										1,
										2)
																
				text = "angle: {}".format(self.angle)
				
				cv_image = cv2.putText(cv_image, 
										text,
										(3, 44),
										cv2.FONT_HERSHEY_SIMPLEX,
										0.4,
										(255, 0, 155),
										1,
										2)
			
			# Publish the image with text
			self.img_pub.publish(self.cvBridge.cv2_to_imgmsg(cv_image, encoding))
		else:
			self.img_pub.publish(img)
	
	# Set the mode based on the passed data
	def cb_front_mode(self, data):
		self.front_mode = data.data

	# Set if info on the image needs to be outputted
	def cb_switch_env(self, data):
		self.show_env = data.data
	
	def main(self):
		if not self.plan:
			rospy.signal_shutdown('force ending')
			return


if __name__ == '__main__':
	rospy.init_node('control', disable_signals = True)
	try:
		node = Xlidarcontrol()
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
		pass

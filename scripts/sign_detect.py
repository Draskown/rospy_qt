#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Needed imports
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os
from time import sleep
from std_msgs.msg import String, Bool


# Compares the got matches
def compare_matches(kp,kp_ideal,matches):
	MATCHES_ERR = 50000
	MATCHES_DIST_MIN = 7
	
	good = []
	for m,n in matches:
		if m.distance < 0.7*n.distance:
			good.append(m)
	
	if len(good) > MATCHES_DIST_MIN:
		src_pts = np.float32([kp[m.queryIdx].pt for m in good])
		dst_pts = np.float32([kp_ideal[m.trainIdx].pt for m in good])
	
		mse_err = find_mse(src_pts,dst_pts)
		
		if mse_err < MATCHES_ERR:
			return True
	
	return False

# Finds the error between images
def find_mse(arr1, arr2):
	err = (arr1-arr2)**2
	sum_err = err.sum()
	size = arr1.shape[0]
	sum_err = sum_err/size
	return sum_err
	
# Initializes the signs
def standart_signs():
	dir_path = os.path.dirname(os.path.realpath(__file__))
	dir_path += '/data_set/'
	
	img1 = cv2.imread(dir_path + 'stop.png',0)
	img2 = cv2.imread(dir_path + 'parking.png',0)
	img3 = cv2.imread(dir_path + 'tunnel.png',0)    
	
	sift = cv2.SIFT_create()
	kp1,des1 = sift.detectAndCompute(img1, None)
	kp2, des2 = sift.detectAndCompute(img2,None)
	kp3, des3 = sift.detectAndCompute(img3,None)
	
	kp_ideal = [kp1,kp2,kp3]
	des_ideal = [des1,des2,des3]
	return kp_ideal, des_ideal, sift


class SignDetect():
	def __init__(self):
		# Initializing global values
		self.cvBridge = CvBridge()
		self.counter = 1
		self.use_signs = True
		self.sign_msg = String()
		self.plan = True
		self.kp_ideal, self.des_ideal, self.sift = standart_signs()
		
		# Initializing of a Flann Matcher
		FLANN_INDEX_KDTREE = 0
		index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
		search_params = dict(checks = 50)
		self.flann = cv2.FlannBasedMatcher(index_params, search_params)

		# Set publishers for image of signs and detected sign
		self.pub_image = rospy.Publisher('sign_image', Image, queue_size=1)
		self.pub_sign = rospy.Publisher('sign', String, queue_size=1)
		self.pub_log = rospy.Publisher('log_msg', String, queue_size=1)

		# Set subscribers for the ROS topics
		sub_image = rospy.Subscriber('/camera/image', Image, self.cbImageProjection, queue_size=1)
		sub_ts = rospy.Subscriber('state', String, self.cb_ts, queue_size=1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)
		sub_start = rospy.Subscriber('start_mission', Bool, self.cbStart, queue_size = 1)

		# While ros is not shut down - handle every subscriber's method
		while not rospy.is_shutdown():
			try:
				if self.plan:
					rospy.sleep(0.1)
				else:
					rospy.signal_shutdown('force ending')
					cv2.destroyAllWindows()
					break
			except KeyboardInterrupt:
				cv2.destroyAllWindows()
				break			
	
	def cbStart(self, data):
		self.use_signs = not data.data

	# Listens fot the plan's state
	def cbPlan(self, data):
		self.plan = data.data

	# Listens for the robot's state
	def cb_ts(self, data):
		if int(data.data) >= 4:			
			self.use_signs = True

	# Listens for the camera's image
	def cbImageProjection(self, data):
		if self.use_signs == False:
			return
		
		# Drop the frame to 1/5 (6fps) because of the processing speed
		if self.counter % 3 != 0:
			self.counter += 1
			return
		else:
			self.counter = 1
		
		# Preparations for the image
		cv_image_original = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
		cv_image_gray = cv2.cvtColor(cv_image_original, cv2.COLOR_BGR2GRAY)
		kp,des = self.sift.detectAndCompute(cv_image_gray, None)
		cv_image_original = cv2.drawKeypoints(cv_image_gray,kp,None,(255,0,0),4)
		
		# Compare the image to every sign
		for i in range(0,3):
			matches = self.flann.knnMatch(des,self.des_ideal[i],k=2)
			result = compare_matches(kp, self.kp_ideal[i], matches)
			if result == True:
				if i == 0:
					self.sign_msg.data = "stop"
					self.pub_log.publish("Stop detected \r\n")
				elif i == 1:
					self.sign_msg.data = "parking"
				else:
					self.sign_msg.data = "tunnel"
				break
			else:
				self.sign_msg.data = "none"
		
		# Publish the got image and detected sign
		self.pub_sign.publish(self.sign_msg)
		self.pub_image.publish(self.cvBridge.cv2_to_imgmsg(cv_image_original, "rgb8"))


if __name__ == '__main__':
	rospy.init_node('sign_detect')
	try:
		signdetect = SignDetect()
	except rospy.ROSInterruptException:
		pass

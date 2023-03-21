#!/usr/bin/env python3
#-*- coding:utf-8 -*-

# Needed imports
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import String, Bool

# Masks the image with yellow
def mask_yellow(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 20
	Hue_h = 35
	Saturation_l = 100
	Saturation_h = 255
	Lightness_l = 50
	Lightness_h = 255

# Define range of yellow color in HSV
	lower_yellow = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
	temp = np.hsplit(mask,2)
	mask = temp[1]
	fraction_num = np.count_nonzero(mask)
	if fraction_num > 100:
		for y in range(0, mask.shape[0]-1,5):
			for x in range(0,mask.shape[1]-1,5):
				if mask[y,x]>0:
					return(x+160,y) 
	return 0, 0

# Masks the image with red
def mask_red(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 0
	Hue_h = 10
	Saturation_l = 30
	Saturation_h = 255
	Lightness_l = 48
	Lightness_h = 255
	
	# define range of red color in HSV
	lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_red = np.array([Hue_h, Saturation_h, Lightness_h])
	
	# Threshold the HSV image to get only red colors
	mask = cv2.inRange(hsv, lower_red, upper_red)
	temp = np.hsplit(mask,2) 
	mask = temp[1]
	fraction_num = np.count_nonzero(mask)
	if fraction_num > 100:
		for y in range(0, mask.shape[0]-1,5):
			for x in range(0,mask.shape[1]-1,5):
				if mask[y,x]>0:
					return(x+160,y)
	return 0, 0

# Masks the image with green
def mask_green(img):
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 46
	Hue_h = 76
	Saturation_l = 86
	Saturation_h = 255
	Lightness_l = 50
	Lightness_h = 255
	
	# define range of green color in HSV
	lower_green = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_green = np.array([Hue_h, Saturation_h, Lightness_h])
	
	# Threshold the HSV image to get only green colors
	mask = cv2.inRange(hsv, lower_green, upper_green)
	temp = np.hsplit(mask,2)
	mask = temp[1]
	fraction_num = np.count_nonzero(mask)
	if fraction_num > 100:
		for y in range(0, mask.shape[0]-1,5):
			for x in range(0,mask.shape[1]-1,5):
				if mask[y,x]>0:
					return(x+160,y) 
	return 0, 0

# Calculates the error of detection
def calc_err(color_x, color_y, circle_x, circle_y):
	if color_x*color_y*circle_y*circle_x > 0:
		x_err = (color_x - circle_x)**2
		y_err = (color_y - circle_y)**2
		err = x_err+y_err / 2
		return err
	return 100000


class TlDetector():
	def __init__(self):
		# Initializng of the global values
		self.cvBridge = CvBridge()
		self.counter = 1
		self.light_msg = String()
		self.plan = True

		# Set the publishers for tl's image and state
		self.pub_image = rospy.Publisher('image_traffic_light', Image, queue_size=1)
		self.pub_traffic_light = rospy.Publisher('traffic_light', String, queue_size=1)
		
		# Set subscribers for the ROS topics
		sub_image = rospy.Subscriber('/camera/image', Image, self.cbImageProjection, queue_size=1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)

		# While ROS is not shut down and tl is not detected - listen for the detection
		while not rospy.is_shutdown():
				try:
					if self.light_msg.data != "green" and self.plan:
						rospy.sleep(0.1)
					else:
						cv2.destroyAllWindows()
						break
				except KeyboardInterrupt:
					cv2.destroyAllWindows()
					break

	# Listens for the plan's state
	def cbPlan(self, data):
		self.plan = data.data

	# Listens for the camera's image
	def cbImageProjection(self, data):
		# Drop the frames to 1/5
		if self.counter % 3 != 0:
			self.counter += 1
			return
		else: 
			self.counter = 1  
		
		# Preparations for the image processing
		cv_image_original = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
		cv_image_original = cv2.GaussianBlur(cv_image_original, (3, 3), 0)
		cv_image_gray = cv2.cvtColor(cv_image_original, cv2.COLOR_BGR2GRAY)
		
		# Initializing of Hugh Cricles to detect the traffic light
		circles = cv2.HoughCircles(cv_image_gray, cv2.HOUGH_GRADIENT, 1, 50, param2 = 20, minRadius = 5, maxRadius = 15 ) 
		green_x, green_y  = mask_green(cv_image_original)
		yellow_x, yellow_y = mask_yellow(cv_image_original)
		red_x, red_y  = mask_red(cv_image_original)
		
		# Initialize the tl's data
		self.light_msg.data = "none"
		
		if circles is not None:
			circles = np.round(circles[0,:]).astype("int")
			for x,y,r in circles:
				cv2.circle(cv_image_gray, (x,y),r,(0,255,0), 7)
				green_err = calc_err(green_x, green_y, x, y) 
				red_err = calc_err(red_x, red_y, x, y)
				yellow_err = calc_err(yellow_x, yellow_y, x, y)
				
				if green_err < 400:
					self.light_msg.data = "green"
				if red_err < 400:
					self.light_msg.data = "red"
				if yellow_err < 400:
					self.light_msg.data = "yellow"
		
		# Publish the proccessed image and detected tl
		self.pub_traffic_light.publish(self.light_msg)
		temp = np.hsplit(cv_image_gray,2) 
		cv_image_gray = temp[1]
		self.pub_image.publish(self.cvBridge.cv2_to_imgmsg(cv_image_gray, "8UC1"))
	
	
if __name__ == '__main__':
	rospy.init_node('traffic_light_detector')
	try:
		tldetector = TlDetector()
	except rospy.ROSInterruptException:
		pass

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Needed imports
import rospy, cv2, numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import Float64, Bool, Int8


# Masks the image with yellow
def mask_yellow(img):
	# Parameters of the mask
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 27
	Hue_h = 41
	Saturation_l = 130
	Saturation_h = 255
	Lightness_l = 160
	Lightness_h = 255

	# Define range of yellow color in HSV
	lower_yellow = np.array([Hue_l, Saturation_l, 	Lightness_l])
	upper_yellow = np.array([Hue_h, Saturation_h, Lightness_h])

	# Threshold the HSV image to get only yellow colors
	mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

	# Bitwise-AND mask and original image
	mask_res = cv2.bitwise_and(img, img, mask = mask)
	res = cv2.bitwise_and(img, img, mask = mask)
	fraction_num = np.count_nonzero(mask)
	
	point_arr = []
	stop_flag = False

	# Generate a vector of yellow points
	if fraction_num > 50:
		k = 0
		jold = 0
		for i in range(mask.shape[0]-1,0,-15):
			if stop_flag == True:
				break
			for j in range(0, mask.shape[1], 15):
				if mask[i,j] > 0:
					point_arr.append([j,i])
					k+=1
					if abs(j-jold) > 80 and k > 1:
						point_arr.pop()
						stop_flag = True
					jold = j
					break
		
		# Draw a vector of yellow points
		if(len(point_arr) > 0):
			point_before = point_arr[0]
			for point in point_arr:
				res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
				point_before = point
				
	return res, point_arr, mask_res

# Same as 'mask_yellow'
def mask_white(img):
		# Parameters of the mask
		hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
		Hue_l = 0
		Hue_h = 25
		Saturation_l = 0
		Saturation_h = 36
		Lightness_l = 180
		Lightness_h = 255

		# Define range of yellow color in HSV
		lower_white = np.array([Hue_l, Saturation_l, Lightness_l])
		upper_white = np.array([Hue_h, Saturation_h, Lightness_h])

		# Threshold the HSV image to get only white colors
		mask = cv2.inRange(hsv, lower_white, upper_white)

		# Bitwise-AND mask and original image
		res_mask = cv2.bitwise_and(img, img, mask = mask)
		res = cv2.bitwise_and(img, img, mask = mask)
		fraction_num = np.count_nonzero(mask)

		point_arr = []
		stop_flag = False

		# Generate a vector of white points
		if fraction_num > 50:
			k = 0
			jold = 0
			for i in range(mask.shape[0]-1,0,-20):
				if stop_flag == True:
					break
				for j in range(mask.shape[1]-1,0,-20):
					if mask[i,j] > 0:
						point_arr.append([j,i])
						k+=1
						if abs(j-jold) > 80 and k > 1:
							point_arr.pop()
							stop_flag = True
						jold = j
						break
			
			# Draw the vector of white points
			if len(point_arr) > 0:
				point_before = point_arr[0]
				for point in point_arr:
					res = cv2.line(res, (point[0], point[1]), (point_before[0],point_before[1]), (0,0,255),8)
					point_before = point
				
		return res, point_arr, res_mask

# Calculates the error between the lines
def calculate_error(yellow_array, white_array):
	error_yell = 0
	error_white = 0
	weight = 0
	i = 1

	for yel in yellow_array:
	# When yel[2] = 600 then weight = 0 and if yel[2] = 0 wheight = 1
		weight = yel[1]*0.0017 + 1
		error_yell = weight*(20 - yel[0]) + error_yell
		i+=1
	error_yell = error_yell/i
	for white in white_array:
		weight = white[1]*0.0017 + 1
		error_white = weight*(290 - white[0]) + error_white
		i+=1
	error_white = error_white/i
	
	if error_white < 30:
		return error_yell
	elif error_yell < 30:
		return error_white
	else:
		return (error_white + error_yell)/2


class LineDetect():
	def __init__(self):
		# Initializing of global variables
		self.cvBridge = CvBridge()
		self.plan = True
		self.mode = 1
		self.show_perspective = False
		self.show_detection = False

		# Set publishers for image and for the error
		self.pub_image = rospy.Publisher('image', Image, queue_size=1)
		self.pub_error = rospy.Publisher('line_error', Float64, queue_size=1)

		# Set subscribers for ROS topics
		sub_image = rospy.Subscriber('/camera_line/image', Image, self.cbImageProjection, queue_size = 1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)
		line_sub = rospy.Subscriber('line_camera_mode', Int8, self.cb_mode, queue_size=1)
		show_detection_sub = rospy.Subscriber('show_line_det', Bool, self.cb_show_detection, queue_size=1)
		show_perspective_sub = rospy.Subscriber('show_persp', Bool, self.cb_show_perspective, queue_size=1)

		# Do every subscriber's method until ros is shut down
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

	# Listens for the plan's state
	def cbPlan(self, data):
		self.plan = data.data

	# Listens for the original image
	def cbImageProjection(self, data):
		# Get and prepare the original camera's image
		cv_image_original = self.cvBridge.imgmsg_to_cv2(data, "bgr8")	
		cv_image_blur = cv2.GaussianBlur(cv_image_original, (5, 5), 0)
		
		if self.show_perspective:
			pass

		# Mask the image with white and yellow
		yellow_detect, yellow_array, yellow_mask = mask_yellow(cv_image_blur)
		white_detect, white_array, white_mask = mask_white(cv_image_blur)

		# Add detected values onto the image
		if self.mode == 1:
			detected = cv_image_original.copy()
			if self.show_detection:
				detected = cv2.add(white_detect, yellow_detect)
		elif self.mode == 2:
			detected = white_mask.copy()
			if self.show_detection:
				detected = white_detect.copy()
		elif self.mode == 3:
			detected = yellow_mask.copy()
			if self.show_detection:
				detected = yellow_detect.copy()

		# Publish the image into the topic		
		self.pub_image.publish(self.cvBridge.cv2_to_imgmsg(detected, "bgr8"))

		# Generate and publish the calcualated error from the lines
		error = Float64()
		error.data = calculate_error(yellow_array, white_array)
		self.pub_error.publish(error)

	def cb_mode(self, data):
		self.mode = data.data

	def cb_show_perspective(self, data):
		self.show_perspective = data.data

	def cb_show_detection(self, data):
		self.show_detection = data.data

if __name__ == '__main__':
	rospy.init_node('image_projection')
	try:
		linedetect = LineDetect()
	except rospy.ROSInterruptException:
		pass

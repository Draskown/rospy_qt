#!/usr/bin/env python3
#-*- coding:utf-8 -*-

# Needed imports
import rospy, numpy as np, cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from time import sleep
from std_msgs.msg import Bool, String


# Applies the red mask to the image
def mask_red(img):
	# Initializing of a colorspace and thresholds
	hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	Hue_l = 0
	Hue_h = 10
	Saturation_l = 30
	Saturation_h = 255
	Lightness_l = 48
	Lightness_h = 255
	
	# Define range of red color in HSV
	lower_red = np.array([Hue_l, Saturation_l, Lightness_l])
	upper_red = np.array([Hue_h, Saturation_h, Lightness_h])
	
	# Threshold the HSV image to get only red colors
	mask = cv2.inRange(hsv, lower_red, upper_red)

	# For detection of a big rectangle
	mask = cv2.erode(mask, (4,4), iterations = 6) 
	
	# Count white pixels on the applied mask
	fraction_num = np.count_nonzero(mask)
	
	if fraction_num > 150:
		return True, mask
	else:
		return False, mask


class BarDetect():
	def __init__(self):
		# Initializing of global variables
		self.pub_bar = rospy.Publisher('bar', String, queue_size=1)
		self.cvBridge = CvBridge()
		self.counter = 1
		self.enable = False
		self.plan = True

		# Set a publisher for an image
		self.pub_image = rospy.Publisher('image_bar', Image, queue_size=1)
		
		# Set subscribers for ROS topics
		sub_image = rospy.Subscriber('/camera/image', Image, self.cbImageProjection, queue_size=1)
		sub_tl = rospy.Subscriber('state', String, self.cb_ts, queue_size=1)
		sub_plan = rospy.Subscriber('plan', Bool, self.cbPlan, queue_size = 1)

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


	# Listens for the robot's state
	def cb_ts(self, data):
		if int(data.data) >= 5:
			self.enable = True


	# Prepares an image for detection of red rectangles
	def cbImageProjection(self, data):
		if not self.enable:
			return
		
		# Drop the framerate to 1/5
		if self.counter % 3 != 0:
			self.counter += 1
			return
		else:
			self.counter = 1

		# Read the original image
		cv_image_original = self.cvBridge.imgmsg_to_cv2(data, "bgr8")
		# Blur the image
		cv_image_original = cv2.GaussianBlur(cv_image_original, (3, 3), 0)
		# Generate the message of the bar's detection
		bar_msg = String()

		# Applies the mask to the image
		temp, res = mask_red(cv_image_original)
		
		# If found any red rectangles - return bar state
		if temp:
			bar_msg.data = "bar"
		else:
			bar_msg.data = "none"
		
		# Publish the binary image and the detection's result
		cv_image_rgb = cv2.cvtColor(res, cv2.COLOR_GRAY2RGB)
		self.pub_bar.publish(bar_msg)
		self.pub_image.publish(self.cvBridge.cv2_to_imgmsg(cv_image_rgb, "rgb8"))



if __name__ == '__main__':
	rospy.init_node('bar_detect')
	try:
		node = BarDetect()
	except rospy.ROSInterruptException:
		pass

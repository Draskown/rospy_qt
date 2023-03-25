#!/usr/bin/env python3
#-*- coding:utf-8 -*-

### Made by Fern Lane, apparently
### Huge thanks for the work


# Needed imports
import rospy, math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist


class Encoders():
    def __init__(self):
        # Init global variables
        self.wheel_radius = 0.033
        self.turning_radius = 0.08
        self.last_time = 0.0
        self.position_x = 0.0
        self.position_y = 0.0
        self.direction = 0.0
        self.left_enc = 0.0
        self.right_enc = 0.0

        # Publishers for positions
        self.position_x_publisher = rospy.Publisher('position_x', Float64, queue_size=1)
        self.position_y_publisher = rospy.Publisher('position_y', Float64, queue_size=1)
        self.direction_publisher = rospy.Publisher('direction', Float64, queue_size=1)

        self.enc_r_pub = rospy.Publisher('enc_r', Float64, queue_size=1)
        self.enc_l_pub = rospy.Publisher('enc_l', Float64, queue_size=1)

        # Create ROS topics subscribers
        rospy.Subscriber('cmd_vel', Twist, self.cb_vel, queue_size=1)
        rospy.Subscriber('set_x', Float64, self.set_x, queue_size=1)
        rospy.Subscriber('set_y', Float64, self.set_y, queue_size=1)
        rospy.Subscriber('set_direction', Float64, self.set_direction, queue_size=1)

		# Do every subscriber's method until ros is shut down
        while not rospy.is_shutdown():
            try:
                rospy.sleep(0.1)
            except KeyboardInterrupt:
                rospy.signal_shutdown('force ending')
                break

    # Methods
    # Listens for the robot's velocity
    def cb_vel(self, data):
        # Get current time in seconds
        time_now = rospy.Time.now().to_sec()

        # Calculate time difference
        time_diff = time_now - self.last_time

        # Check if time has passed
        if time_diff > 0.0:
            # Store current time for next cycle
            self.last_time = time_now

            # Get linear and angular velocities from subscriber
            linear_velocity  = data.linear.x
            angular_velocity = data.angular.z

            # Calculate instantaneous velocities for both wheels
            total_velocity_left = linear_velocity - (angular_velocity * self.turning_radius) / self.wheel_radius
            total_velocity_right = linear_velocity + (angular_velocity * self.turning_radius) / self.wheel_radius

            # Calculate instantaneous distance
            distance_left_now = total_velocity_left * time_diff
            distance_right_now = total_velocity_right * time_diff

            # Calculate overall distance
            self.left_enc += distance_left_now
            self.right_enc += distance_right_now

            # Calculate delta s and delta turn
            delta_s = self.wheel_radius * (distance_right_now + distance_left_now) / 2.0
            delta_theta = self.wheel_radius * (distance_right_now - distance_left_now) / (self.wheel_radius * 2.0)

            # Calculate absolute positions
            self.position_x += delta_s * math.cos(self.direction + (delta_theta / 2.))
            self.position_y += delta_s * math.sin(self.direction + (delta_theta / 2.))
            self.direction += delta_theta

            # Publish positions
            self.position_x_publisher.publish(self.position_x)
            self.position_y_publisher.publish(self.position_y)
            self.direction_publisher.publish(self.direction % math.radians(360.))

            # Publish encoders
            self.enc_l_pub.publish(self.left_enc)
            self.enc_r_pub.publish(self.right_enc)


    # Listens for the x position and sets it
    def set_x(self, data):
        self.position_x = data.data

    # Listens for the y position and sets it
    def set_y(self, data):
        self.position_y = data.data

    def set_direction(self, data):
        self.direction = data.data
    ###


if __name__ == '__main__':
    rospy.init_node('encoders')
    try:
        node = Encoders()
    except rospy.ROSInterruptException:
        pass
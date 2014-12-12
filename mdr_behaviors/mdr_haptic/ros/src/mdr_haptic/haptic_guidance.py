#! /usr/bin/env python
import rospy
import tf
import time
import math
import std_srvs.srv
import mcr_algorithms.controller.pid_controller as pid_controller
import mcr_algorithms.filter.low_pass_filter as low_pass_filter
from geometry_msgs.msg import *
from nav_msgs.msg import *

class haptic_guidance():
	def __init__(self):
		# get parameter :
		self.wrench_input = rospy.get_param('/mdr_behaviors/haptic_guidance/wrench_input')
		self.velocity_input = rospy.get_param('/mdr_behaviors/haptic_guidance/velocity_input')
		self.velocity_output = rospy.get_param('/mdr_behaviors/haptic_guidance/velocity_output')

		
		# boolean switch for services
		self.haptic_base = False
		
		# Default movement mode
		self.movement_mode = 'DRIVE' #SHIFT
		
		# Default position of arm 
		# It is actually not the default, but most of the time, the end is behind Jenny
		self.arm_in_front = False
		
		# sensitivity set & zero threshold
		## x parameter
		abs_range_force_x = 30
		abs_range_velocity_x = 0.5
		self.x_velocity_force_normalization = ( abs_range_velocity_x / abs_range_force_x )
		self.maximum_publish_velocity_x = 0.3
		self.bottom_velocity_threshold_x = 0.01
		# self.bottom_velocity_threshold_x = 0.03
		
		## y parameter
		self.set_y_normalization()
		
		# low pass filter
		self.moving_average_points = 50 # base on average publish rate
		self.moving_average_y = low_pass_filter.moving_average(self.moving_average_points)
		self.moving_average_x = low_pass_filter.moving_average(self.moving_average_points)
		
		# Initial position
		self.zero_x = 0.0
		self.zero_y = 0.0
		self.set_zero = False
		
		# PID controller
		self.current_velocity_x = 0.0
		self.current_velocity_y = 0.0
		self.current_velocity_z = 0.0
		self.set_velocity_x = 0.0
		self.set_velocity_y = 0.0
		self.control_velocity_x = 0.0
		self.control_velocity_y = 0.0
		proportional_constant = 0.15
		derivative_constant = 0.2
		sampling_time = 5
		self.x_controller = pid_controller.pd_controller(proportional_constant, derivative_constant, sampling_time)		
		self.y_controller = pid_controller.pd_controller(proportional_constant, derivative_constant, sampling_time)		
		
		# ROS PARAMETER
		self.base_velocity_publisher = rospy.Publisher(self.velocity_output, geometry_msgs.msg.Twist)
		self.base_velocity = geometry_msgs.msg.Twist()
		self.base_velocity.linear.x = 0
		self.base_velocity.linear.y = 0
		self.base_velocity.linear.z = 0
		self.base_velocity.angular.x = 0
		self.base_velocity.angular.y = 0
		self.base_velocity.angular.z = 0
		
		# sampling data for evaluation
		self.time_loop = rospy.Time.now()
		
		# Frame transformation
		self.listener = tf.TransformListener()
		self.force_origin = geometry_msgs.msg.PointStamped()
		self.force_transformed = geometry_msgs.msg.PointStamped()
		
		# advertise services
		start_service = rospy.Service('~start', std_srvs.srv.Empty, self.start)
		stop_service = rospy.Service('~stop', std_srvs.srv.Empty, self.stop)
		shift_service = rospy.Service('~shift', std_srvs.srv.Empty, self.shift)
		drive_service = rospy.Service('~drive', std_srvs.srv.Empty, self.drive)
		
		# subcriber
		rospy.Subscriber(self.velocity_input, nav_msgs.msg.Odometry, self.odometry_callback)
		rospy.Subscriber(self.wrench_input, geometry_msgs.msg.WrenchStamped, self.wrench_callback)
		
		# logging
		self.log = False

		if self.log == True:
			self.log_time = rospy.Time.now()
			print ('delta_x,',
				' normalized_x,',
				' delta_x_normalized,',
				' filtered_x,',
				' current_x.', 
				' controlled_x.', 
				' publish_x,', 
				' delta_y,', 
				' normalized_z,', 
				' filtered_z,', 
				' current_z,', 
				' controlled_z,', 
				' publish_z' )
		
	def wrench_callback(self, msg):
		# transform force to desired frame of reference 
		self.force_origin.header.frame_id = msg.header.frame_id
		self.force_origin.point = msg.wrench.force
		self.listener.waitForTransform('/base_link', '/arm_7_link', rospy.Time(), rospy.Duration(10.0))
		self.force_transformed = self.listener.transformPoint('/base_link',self.force_origin)
		
		# set default position at the initialization of services
		if self.set_zero == False:
			self.zero_x = self.force_transformed.point.x
			self.zero_y = self.force_transformed.point.y
			# check joint 7 position for inverting the value
			self.listener.waitForTransform('/base_link', '/arm_7_link', rospy.Time(), rospy.Duration(10.0))
			try:
				(trans, rot) = self.listener.lookupTransform('/base_link', '/arm_7_link', rospy.Time(0.0))
			except (tf.LookupException, tf.ConnectivityException):
				rospy.loginfo('HAPTIC:failed getting arm_7_link position')
			if trans[0] < 0:
				self.arm_in_front = True
			else:
				self.arm_in_front = False			
			self.set_zero = True
		
		# difference from default position as input
		delta_x = self.force_transformed.point.x - self.zero_x
		delta_y = self.force_transformed.point.y - self.zero_y
		
		# invert the delta
		delta_x = -delta_x
		delta_y = -delta_y
		
		# normalization
		delta_x_normalized = delta_x * self.x_velocity_force_normalization
		delta_y_normalized = delta_y * self.y_velocity_force_normalization
		
		# low pass filter
		self.moving_average_x.add_points(delta_x_normalized)
		self.moving_average_y.add_points(delta_y_normalized)
		self.set_velocity_x = self.moving_average_x.value()
		self.set_velocity_y = self.moving_average_y.value()
		
		# PID controller
		self.control_velocity_x = self.x_controller.control(self.set_velocity_x, self.current_velocity_x)
		if self.movement_mode == 'DRIVE':
			self.control_velocity_y = self.y_controller.control(self.set_velocity_y, -self.current_velocity_z)		
		else:
			self.control_velocity_y = self.y_controller.control(self.set_velocity_y, self.current_velocity_y)
		
		######################
		# SAFETY MEASUREMENT #
		######################
		if(abs(self.control_velocity_x) > self.maximum_publish_velocity_x):
			if self.control_velocity_x > 0 :
				self.control_velocity_x = self.maximum_publish_velocity_x
			else:
				self.control_velocity_x = -self.maximum_publish_velocity_x
		if(abs(self.control_velocity_y) > self.maximum_publish_velocity_y):
			if self.control_velocity_y > 0 :
				self.control_velocity_y = self.maximum_publish_velocity_y
			else:
				self.control_velocity_y = -self.maximum_publish_velocity_y
		
		self.control_no_threshold_x = self.control_velocity_x
		self.control_no_threshold_y = self.control_velocity_y

		# zero threshold
		if(abs(self.control_velocity_y) < self.bottom_velocity_threshold_y):
			self.control_velocity_y = 0.0
		if(abs(self.control_velocity_x) < self.bottom_velocity_threshold_x):
			self.control_velocity_x = 0.0
		
		# map to base_velocity
		self.base_velocity.linear.x = self.control_velocity_x
		if self.movement_mode == 'SHIFT':
			self.base_velocity.linear.y = self.control_velocity_y
		else:
			if self.arm_in_front == True:
				self.control_velocity_y = -self.control_velocity_y
			
			self.base_velocity.angular.z = self.control_velocity_y
		
		# publish velocity
		if self.haptic_base == True:
			time = rospy.Time.now()
			if ( (time) - (self.time_loop) ) > rospy.Duration(0.01):
				self.base_velocity_publisher.publish(self.base_velocity)
				self.time_loop = rospy.Time.now()

		
		# logging	
		if self.log == True:
			time = rospy.Time.now()
			if ( (time) - (self.log_time)) > rospy.Duration(0.1):
				self.log_time = rospy.Time.now()
				print (delta_x ,
					delta_x_normalized, 
					self.set_velocity_x,
					self.current_velocity_x,
					self.control_no_threshold_x, 
					self.base_velocity.linear.x,
					delta_y,
					delta_y_normalized,
					self.set_velocity_y, 
					self.current_velocity_y,
					self.control_no_threshold_y,
					self.base_velocity.angular.z)

					

	def odometry_callback(self, message):
		# get current velocity
		self.current_velocity_x = message.twist.twist.linear.x
		self.current_velocity_y = message.twist.twist.linear.y
		self.current_velocity_z = message.twist.twist.angular.z
		
	def start(self, req):
		# wait for to manipulator to stabilize it pose
		# set the default value
		rospy.sleep(0.5)
		self.set_zero = False
		rospy.sleep(0.5)
		self.time_loop = rospy.Time.now()
		# change mode to default movement
		self.movement_mode = 'DRIVE'
		
		self.set_y_normalization()
		self.haptic_base = True
		rospy.loginfo('haptic guidance: haptic guidance behavior started')
		return std_srvs.srv.EmptyResponse()
		
	def stop(self, req):
		self.haptic_base = False
		rospy.loginfo('haptic guidance: haptic guidance behavior stopped')
		return std_srvs.srv.EmptyResponse()
		
	def shift(self, req):
		self.movement_mode = 'SHIFT'
		self.base_velocity.angular.z = 0.0
		self.set_y_normalization()
		rospy.loginfo("haptic guidance: movement mode changed into 'SHIFT' mode")
		return std_srvs.srv.EmptyResponse()
		
	def drive(self, req):
		self.movement_mode = 'DRIVE'
		self.base_velocity.linear.y = 0.0
		self.set_y_normalization()
		rospy.loginfo("haptic guidance: movement mode changed into 'DRIVE' mode")
		return std_srvs.srv.EmptyResponse()
		
	# normalization, zero threshold for y parameter
	def set_y_normalization(self):		
		if (self.movement_mode == 'SHIFT'):
			abs_range_force_y = 40
			abs_range_velocity_y = 0.5
			self.maximum_publish_velocity_y = 0.4
			self.bottom_velocity_threshold_y = 0.01
		else:
			#self.maximum_publish_velocity_y = 0.4
			#abs_range_force_y = 20
			#abs_range_velocity_y = 0.6
			#self.bottom_velocity_threshold_y = 0.03
			self.maximum_publish_velocity_y = 0.3
			abs_range_force_y = 30
			abs_range_velocity_y = 0.5
			# self.bottom_velocity_threshold_y = 0.01
			self.bottom_velocity_threshold_y = 0.01
		
		self.y_velocity_force_normalization = ( abs_range_velocity_y / abs_range_force_y )
		
def main():
	rospy.init_node('haptic_guidance')
	haptic_guidance()
	rospy.loginfo('haptic guidance: services started')
	rospy.spin()


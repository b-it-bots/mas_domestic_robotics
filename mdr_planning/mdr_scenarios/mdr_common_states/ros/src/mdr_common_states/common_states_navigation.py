#!/usr/bin/python

######################### IMPORTS #########################

import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

import std_msgs.msg
import std_srvs.srv
import actionlib_msgs.msg

from smach_ros import ServiceState

handle_base_non_blocking = 0

class approach_pose(smach.State):

	def __init__(self, pose = "", mode = "omni"):

		smach.State.__init__(
			self,
			outcomes=['success', 'failed'],
			input_keys=['pose', 'message'],
			output_keys=['pose', 'message'])

		self.pose = pose
		self.mode = mode
	        self.speak_pub = rospy.Publisher('/sound/say', std_msgs.msg.String, latch=True)

		# This state moves the robot to the given pose.

	def execute(self, userdata):

		# determine target position
		if self.pose != "":
			pose = self.pose
		elif type(userdata.pose) is str:
			pose = userdata.pose
		elif type(userdata.pose) is list:
			pose = []
			pose.append(userdata.pose[0])
			pose.append(userdata.pose[1])
			pose.append(userdata.pose[2])
		else: # this should never happen
			userdata.message = []
			userdata.message.append(5)
			userdata.message.append("Invalid userdata 'pose'")
			userdata.message.append(userdata.pose)
			return 'failed'

		# try reaching pose
		handle_base = sss.move("base", pose, False, self.mode)
		move_second = False

		timeout = 0
		while True:
			move_base_state = handle_base.get_state()
		
			if (move_base_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED) and (not move_second):
				# do a second movement to place the robot more exactly
				handle_base = sss.move("base", pose, False, self.mode)
				move_second = True
			elif (move_base_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED) and (move_second):
				userdata.message = []
				userdata.message.append(3)
				userdata.message.append("Pose was succesfully reached")
				return 'success'
			elif move_base_state == actionlib_msgs.msg.GoalStatus.REJECTED or move_base_state == actionlib_msgs.msg.GoalStatus.PREEMPTED or move_base_state == actionlib_msgs.msg.GoalStatus.ABORTED or move_base_state == actionlib_msgs.msg.GoalStatus.LOST:
				 
                                self.speak_pub.publish("I can not reach my target position.")
				rospy.logerr("base movement failed with state: %d", move_base_state)
				return 'failed'
				
			rospy.sleep(0.1)
						

			'''
			# check if service is available
			service_full_name = '/base_controller/is_moving'
			try:
				rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
			except rospy.ROSException, e:
				error_message = "%s"%e
				rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
				return 'failed'
		
			# check if service is callable
			try:
				is_moving = rospy.ServiceProxy(service_full_name,Trigger)
				resp = is_moving()
			except rospy.ServiceException, e:
				error_message = "%s"%e
				rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
				return 'failed'
		
			# evaluate sevice response
			if not resp.success.data: # robot stands still
				if timeout > 10:
					SAY("I can not reach my target position because my path or target is blocked. Please help me!")
					timeout = 0
				else:
					timeout = timeout + 1
					rospy.sleep(1)
			else:
				timeout = 0
			'''

#------------------------------------------------------------------------------------------#

class approach_pose_without_retry(smach.State):

	def __init__(self, pose = "", mode = "omni"):

		smach.State.__init__(
			self,
			outcomes=['success', 'failed'],
			input_keys=['pose', 'message'],
			output_keys=['pose', 'message'])


		sub_move_base = rospy.Subscriber("/move_base/status", actionlib_msgs.msg.GoalStatusArray, self.cb_move_base)
	        self.speak_pub = rospy.Publisher('/sound/say', std_msgs.msg.String, latch=True)
		self.pose = pose
		self.mode = mode

		# This state moves the robot to the given pose.

	def execute(self, userdata):

		# determine target position
		if self.pose != "":
			pose = self.pose
		elif type(userdata.pose) is str:
			pose = userdata.pose
		elif type(userdata.pose) is list:
			pose = []
			pose.append(userdata.pose[0])
			pose.append(userdata.pose[1])
			pose.append(userdata.pose[2])
		else: # this should never happen
			userdata.message = []
			userdata.message.append(5)
			userdata.message.append("Invalid userdata 'pose'")
			userdata.message.append(userdata.pose)
			return 'failed'

		# try reaching pose
		handle_base = sss.move("base", pose, False, self.mode)
		move_second = False

		timeout = 0
		while True:
			if (handle_base.get_state() == 3) and (not move_second):
				# do a second movement to place the robot more exactly
				handle_base = sss.move("base", pose, False, self.mode)
				move_second = True
			elif (handle_base.get_state() == 3) and (move_second):
				userdata.message = []
				userdata.message.append(3)
				userdata.message.append("Pose was succesfully reached")
				return 'success'		

			# check if service is available
			service_full_name = '/base_controller/is_moving'
			try:
				rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3))
			except rospy.ROSException, e:
				error_message = "%s"%e
				rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
				return 'failed'
		
			# check if service is callable
			try:
				is_moving = rospy.ServiceProxy(service_full_name,Trigger)
				resp = is_moving()
			except rospy.ServiceException, e:
				error_message = "%s"%e
				rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
				return 'failed'
		
			# evaluate sevice response
			if not resp.success.data: # robot stands still
				if timeout > 15:
                                        self.speak_pub.publish("I can not reach my target position because my path or target is blocked, I will abort.")
					rospy.wait_for_service('base_controller/stop',10)
					try:
						stop = rospy.ServiceProxy('base_controller/stop',Trigger)
						resp = stop()
					except rospy.ServiceException, e:
						error_message = "%s"%e
						rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
					return 'failed'
				else:
					timeout = timeout + 1
					rospy.sleep(1)
			else:
				timeout = 0

# MIKE: commented out -> self.move_base_status is never used 
	def cb_move_base(self, msg):
		self.move_base_status = msg
		
		
class approach_pose_without_retry_non_blocking(smach.State):
	def __init__(self):

		smach.State.__init__(
			self,
			outcomes=['pose_reached', 'could_not_reach_pose', 'base_is_moving', 'failed'],
			input_keys=['target_pose', 'timer', 'approach_state'],
			output_keys=['timer', 'approach_state'])

		sub_move_base = rospy.Subscriber("/move_base/status", actionlib_msgs.msg.GoalStatusArray, self.cb_move_base)
		self.mode = 'diff'
		
	def execute(self, userdata):
		global handle_base_non_blocking
		## init appraoch
		if userdata.approach_state == 0:
			# determine target position
			if userdata.target_pose == '':
				return 'could_not_reach_pose'
			elif type(userdata.target_pose) is str:
				pose = userdata.target_pose
			elif type(userdata.target_pose) is list:
				pose = []
				pose.append(userdata.target_pose[0])
				pose.append(userdata.target_pose[1])
				pose.append(userdata.target_pose[2])

			# try reaching pose
			handle_base_non_blocking = sss.move("base", pose, False, self.mode)
			userdata.approach_state = 1
			return 'base_is_moving'
		
		elif userdata.approach_state == 1:	
			if handle_base_non_blocking.get_state() == 3:
				userdata.approach_state = 0
				userdata.timer = 0
				return 'pose_reached'		

			# check if service is available
			service_full_name = '/base_controller/is_moving'
			try:
				rospy.wait_for_service(service_full_name,rospy.get_param('server_timeout',3)) 
			except rospy.ROSException, e:
				error_message = "%s"%e
				rospy.logerr("<<%s>> service not available, error: %s",service_full_name, error_message)
				userdata.approach_state = 0
				userdata.timer = 0
				return 'failed'
		
			# check if service is callable
			try:
				is_moving = rospy.ServiceProxy(service_full_name,Trigger) 
				resp = is_moving()
			except rospy.ServiceException, e:
				error_message = "%s"%e
				rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
				userdata.approach_state = 0
				userdata.timer = 0
				return 'failed'
		
			# evaluate sevice response
			if not resp.success.data: # robot stands still 
				if userdata.timer > 15: 
                                        self.speak_pub.publish("I can not get there, I will abort.")
					print 'COULD NOT GET TO POS'
					rospy.wait_for_service('base_controller/stop',10)
					try:
						stop = rospy.ServiceProxy('base_controller/stop',Trigger)
						resp = stop() 
					except rospy.ServiceException, e:
						error_message = "%s"%e
						rospy.logerr("calling <<%s>> service not successfull, error: %s",service_full_name, error_message)
					userdata.approach_state = 0
					userdata.timer = 0
					return 'failed'
				else:
					userdata.timer = userdata.timer + 1
					rospy.sleep(1)
					print 'INC TIMER'
					return 'base_is_moving'
			else:
				userdata.timer = 0
				print 'BASE is moving'
				return 'base_is_moving'

# MIKE: commented out -> self.move_base_status is never used 
	def cb_move_base(self, msg):
		self.move_base_status = msg
				
		

###############!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! MODIFY FOR RoboCup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


## Deals with direct movements of the base (without planning and collision checking).
#
# A target will be sent directly to the base_controller node.
#
def move_base_direct_robocup():
	pub_base = rospy.Publisher('/base/twist_mux/command_navigation', Twist)	
	
	rospy.loginfo("Move base to <<%s>>", parameter_name)
	
	# check trajectory parameters
	if not type(param) is list: # check outer list
			rospy.logerr("no valid parameter for base: not a list, aborting...")
			print "parameter is:",param
			return 
	else:
		#print i,"type1 = ", type(i)
		DOF = 4
		if not len(param) == DOF: # check dimension
			rospy.logerr("no valid parameter for base: dimension should be %d and is %d, aborting...",DOF,len(param))
			print "parameter is:",param
			return 
		else:
			for i in param:
				#print i,"type2 = ", type(i)
				if not ((type(i) is float) or (type(i) is int)): # check type
					#print type(i)
					rospy.logerr("no valid parameter for base: not a list of float or int, aborting...")
					print "parameter is:",param
					return 
				else:
					rospy.logdebug("accepted parameter %f for base",i)

	# sending goal
	twist = Twist()
	twist.linear.x = param[0]
	twist.linear.y = param[1]
	twist.angular.z = param[2]

	loop_rate = 0.1

	for i in range((param[3]/loop_rate)):
		rospy.sleep(loop_rate)	
		pub_base.publish(twist)
	

	# stop base
	twist.linear.x = 0
	twist.linear.y = 0
	twist.angular.z = 0
	pub_base.publish(twist)
	

	return

class direct_base_timed(smach.State):
	def __init__(self, timeout):
		smach.State.__init__(self, outcomes=['success'])
                self.pub_vel = rospy.Publisher('/base/twist_mux/command_navigation', Twist)
                self.timeout = timeout

	def execute(self, userdata):
            timeout = rospy.Duration.from_sec(self.timeout)
            rate = rospy.Rate(5)
            start_time = rospy.Time.now()
            t = Twist()
            t.linear.x = 0.1
            while (rospy.Time.now() - start_time) < timeout:
                self.pub_vel.publish(t)
                rate.sleep()
            stop_t = Twist()
            self.pub_vel.publish(stop_t)

            return 'success'

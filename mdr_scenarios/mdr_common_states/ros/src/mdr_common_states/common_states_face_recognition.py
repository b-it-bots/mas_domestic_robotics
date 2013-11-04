#!/usr/bin/python

######################### IMPORTS #########################

import roslib
roslib.load_manifest('mdr_common_states')

import rospy
import smach
import smach_ros

import std_msgs.msg
import std_srvs.srv
import actionlib_msgs.msg

import mcr_perception_msgs.srv

from mdr_common_states.common_states_speech import *

from simple_script_server import *
sss = simple_script_server()


class is_person_in_front_with_face_rec(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['failed', 'success'])
		self.face_localization = rospy.ServiceProxy('/mcr_perception/face_recognition/start_face_localization', std_srvs.srv.Empty)

	def execute(self,userdata):
		# check whether there is a person in front or not
		is_person_in_front_handle = rospy.ServiceProxy('/mcr_perception/face_recognition/is_person_in_front', std_srvs.srv.Empty) # TODO topic to service does not exist anymore
		rospy.wait_for_service('/mcr_perception/face_recognition/is_person_in_front', 30)
		rospy.wait_for_service('/mcr_perception/face_recognition/start_face_localization', 3)
	
		sss.move("head","front_face")
		try:
			self.face_localization(True)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

		while(True):
			try:
				res = is_person_in_front_handle()
			except rospy.ServiceException,e:
				rospy.logerr("Service call failed: %s", e)
				return 'failed'
			if res.value == True:
				break
			else:
				rospy.sleep(0.1)

		try:
			self.face_localization(False)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
		
		#just to clear topic to service buffer
		try:
			is_person_in_front_handle()
		except rospy.ServiceException,e:
			rospy.logerr("Service call failed: %s", e)
			
		return 'success'
		

class learn_face(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['understood_command'])
		

	def execute(self, userdata):

		sss.move("head","front_face")
		SAY("Please come as close as possible to me and look into my eyes. I will learn your face now.")
		learn_face_handle = rospy.ServiceProxy('/mcr_perception/face_recognition/learn_face', mcr_perception_msgs.srv.SetFaceName)
		face_recognition = rospy.ServiceProxy('/mcr_perception/face_recognition/start_face_recognition', std_srvs.srv.Empty)
		rospy.wait_for_service('/mcr_perception/face_recognition/learn_face', 3)
		rospy.wait_for_service('/mcr_perception/face_recognition/start_face_recognition', 3)
		
		try:
			face_recognition(True)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
		
		try:
			learn_face_handle(userdata.understood_command)
			SAY("I will remember you as " + userdata.understood_command)
			#rospy.sleep(1)
		except rospy.ServiceException, e:
			rospy.logerr("Service call failed: %s", e)
			return 'failed'
			
		try:
			face_recognition(False)
			return 'success'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'


class recognize_person(smach.State):
	def __init__(self, tilt_camera = True):
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['recognized_person_name'], output_keys=['recognized_person_name'])
		self.get_name = rospy.ServiceProxy('/mcr_perception/face_recognition/get_last_face_name', mcr_perception_msgs.srv.GetFaceName)
		self.tilt_camera = tilt_camera				
			
	def execute(self, userdata):
		# recognize person
		# say name
		#######################################################
		face_recognition = rospy.ServiceProxy('/mcr_perception/face_recognition/start_face_recognition', std_srvs.srv.Empty)

		# clearing	
		self.get_name()

		if self.tilt_camera == True:
			sss.move("head","front_face")

		SAY("Please come as close as possible to me and I will try to recognize you.")

		rospy.wait_for_service('/mcr_perception/face_recognition/get_last_face_name', 3)
		try:
			face_recognition(True)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
			
		#check 100 times face detection and if its known
		for i in range(0,200):
			try:
				ret = self.get_name()
			except rospy.ServiceException,e:
				print "Service call failed: %s"%e
				return 'failed'
				

			if ((ret.value != 'unknown') and (ret.value != "")):
				text_to_say = "Hello " + ret.value + ", how are you?"
				userdata.recognized_person_name = ret.value
				print ret.value
				SAY(text_to_say)
				#switch face rec off
				try:
					face_recognition(False)
				except rospy.ServiceException, e:
					print "Service call failed: %s"%e
				#just to clear topic to service buffer
				try:
					self.get_name()
				except rospy.ServiceException,e:
					print "Service call failed: %s"%e
				return 'success'
			else:
				rospy.sleep(0.1)

		# exit loop
		SAY("I don't know you.")
		userdata.recognized_person_name = "unknown"
		#switch face rec off
		try:
			face_recognition(False)
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
			
		#just to clear topic to service buffer
		try:
			self.get_name()
		except rospy.ServiceException,e:
			print "Service call failed: %s"%e
			
		return 'failed'

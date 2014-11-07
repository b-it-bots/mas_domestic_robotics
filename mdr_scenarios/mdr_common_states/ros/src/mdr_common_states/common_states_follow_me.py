#!/usr/bin/python

######################### IMPORTS #########################
import rospy
import smach
import smach_ros

from mdr_common_states.common_states_speech import *

from simple_script_server import *
sss = simple_script_server()

import std_srvs.srv

class init_followme(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'])

	def execute(self, userdata):
		scanner_client = rospy.ServiceProxy('/mcr_drivers/virtual_laser_scanner/start', std_srvs.srv.Empty)
		rospy.wait_for_service('/mcr_drivers/virtual_laser_scanner/start', 3)
		try:
			scanner_client()
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

		# init the head to track person
		head_handle = sss.move("head", "front")		
		sss.move("torso", "home")
	
		tracker_client = rospy.ServiceProxy('/mcr_perception/waist_tracking/start', std_srvs.srv.Empty)
		rospy.wait_for_service('/mcr_perception/waist_tracking/start', 3)
		

		# command the operator
		SAY("Please stand half a meter in front of me")
		followme_client = rospy.ServiceProxy('/mcr_behaviors/follow_person/start', std_srvs.srv.Empty)
		rospy.wait_for_service('/mcr_behaviors/follow_person/start', 3)
		
		try:
			tracker_client()
			followme_client()
			SAY("I'm ready to follow you now!")
			return 'success'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

class stop_following(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'])
	
	def execute(self,userdata):
		followme_client_pause = rospy.ServiceProxy('/mcr_behaviors/follow_person/stop', std_srvs.srv.Empty)
		rospy.wait_for_service('/mcr_behaviors/follow_person/stop', 3)
		
		tracker_client = rospy.ServiceProxy('/mcr_perception/waist_tracking/stop', std_srvs.srv.Empty)
		rospy.wait_for_service('/mcr_perception/waist_tracking/stop', 3)
		
		scanner_client = rospy.ServiceProxy('/mcr_drivers/virtual_laser_scanner/stop', std_srvs.srv.Empty)
		rospy.wait_for_service('/mcr_drivers/virtual_laser_scanner/stop', 3)
		try:
			followme_client_pause()
			tracker_client()
			scanner_client()
			SAY("I stopped following.")
			return 'success'
		except:
			print "Service call failed: %s"%e
			return 'failed'

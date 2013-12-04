#!/usr/bin/python

import rospy
import smach
import smach_ros

import std_srvs.srv
import mcr_perception_msgs.msg

from mdr_common_states.common_states import *


class init_manipulator(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'])
		self.recover_srv = rospy.ServiceProxy('/arm_controller/lwr_node/recover', std_srvs.srv.Empty)

	def execute(self, userdata):
		rospy.wait_for_service('/arm_controller/lwr_node/recover')
		recover = rospy.ServiceProxy('/arm_controller/lwr_node/recover', std_srvs.srv.Empty)
		try:
			recover()
		except rospy.ServiceException, e:
			print "Service did not process request: %s"%str(e)

		return 'success'
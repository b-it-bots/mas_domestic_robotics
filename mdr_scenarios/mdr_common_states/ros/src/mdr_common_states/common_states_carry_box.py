#!/usr/bin/python

######################### IMPORTS #########################
import rospy
import smach
import smach_ros
import tf
import moveit_commander

import std_msgs.msg
import std_srvs.srv
import actionlib_msgs.msg
import manipulation_msgs.msg

from simple_script_server import *
sss = simple_script_server()

import mcr_perception_msgs.msg
#import mdr_manipulation_msgs.msg

from mdr_common_states.common_states_speech import *


class detect_beer_box(smach.State):

	def __init__(self, beer_box_height_in_meter = 0.35):
		smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['grasp_pose'], output_keys=['grasp_pose'])
	
		self.nearest_object_start = rospy.ServiceProxy('/mcr_perception/nearest_object_detector/start', std_srvs.srv.Empty)
		self.nearest_object_stop = rospy.ServiceProxy('/mcr_perception/nearest_object_detector/stop', std_srvs.srv.Empty)
		self.nearest_object_get = rospy.ServiceProxy('/mcr_perception/nearest_object_detector/GetNearestObject', mcr_perception_msgs.srv.GetNearestObject)

		self.beer_box_height_in_meter = beer_box_height_in_meter
	def execute(self, userdata):
		sss.move("head", "back_table", False)
		sss.move("torso", "extrem_back")
	
		rospy.wait_for_service('/mcr_perception/nearest_object_detector/start', 5)
		rospy.wait_for_service('/mcr_perception/nearest_object_detector/stop', 5)
		rospy.wait_for_service('/mcr_perception/nearest_object_detector/GetNearestObject', 5)
				
		try:
			self.nearest_object_start()
			rospy.sleep(2)
			
			obj_pose = self.nearest_object_get()
			print "object pose before: ", userdata.grasp_pose
			
			userdata.grasp_pose = obj_pose.pose
			userdata.grasp_pose.position.z = self.beer_box_height_in_meter
			print "object pose: ", userdata.grasp_pose
			self.nearest_object_stop()
			return 'success'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.nearest_object_stop()
			return 'failed'

class grasp_beer_box(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'], input_keys=['grasp_pose'], output_keys=['grasp_pose'])
		
		self.grasp = rospy.ServiceProxy('/mcr_manipulation/grasp_crate/grasp_crate', mdr_manipulation_msgs.srv.Pickup)
	
	def execute(self, userdata):
		rospy.wait_for_service('/mcr_manipulation/grasp_crate/grasp_crate', 5)
		
		try:
			req = mdr_manipulation_msgs.srv.PickupRequest()
			req.object.pose.header.frame_id = "/base_link"
			req.object.pose.header.stamp = rospy.Time.now()
			req.object.pose.pose = userdata.grasp_pose
			
			q = userdata.grasp_pose.orientation
			(r, p, y) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
			quat = tf.transformations.quaternion_from_euler(-math.pi, 0.0, -math.pi / 2.0 + y)
			req.object.pose.pose.orientation.x = quat[0]
			req.object.pose.pose.orientation.x = quat[1]
			req.object.pose.pose.orientation.x = quat[2]
			req.object.pose.pose.orientation.x = quat[3]
			
			print req
			
			res = self.grasp(req)
			
			if (res.manipulation_result == object_manipulation_msgs.msg.ManipulationResult.SUCCESS):
				return 'success'
			else:
				return 'failed'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			self.nearest_object_stop()
			return 'failed'

class wait_for_box_in_hand(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'])
		self.get_release_request = rospy.ServiceProxy('/is_external_force_applied', Trigger)
		self.memorize_current_force = rospy.ServiceProxy('/memorize_current_force', std_srvs.srv.Empty)
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		self.arm.set_named_target("guide_beerbox")
		self.arm.go()
		sss.move("sdh","cylopen",False)
		SAY("Please give me the box")
		
		rospy.wait_for_service('/memorize_current_force', 5)
		try:
			self.memorize_current_force()
			rospy.sleep(0.1)
		except rospy.ServiceException,e:
			print "Service call failed: %s"%e
			return 'failed'
			
		# wait for releasing the bottle
		release_request = False
		while not release_request:
			rospy.wait_for_service('/is_external_force_applied', 5)
			try:
				release_request = self.get_release_request().success.data
			except rospy.ServiceException,e:
				print "Service call failed: %s"%e
				return 'failed'
			if release_request: 
				print "grasp object now"
			rospy.loginfo("waiting for grasp request")
			rospy.sleep(0.1)
				
			# check for z axis
			#rospy.wait_for_service('/get_wrench', 5)
			#try:
			#	res = self.get_wrench()
			#except rospy.ServiceException,e:
			#	print "Service call failed: %s"%e
			#	return 'failed'
			#if abs(res.wrench.wrench.force.z) > 10:
			#	print "release z axis"
			#	release_request = True
			
		
		handle_sdh = sss.move("sdh","cylclosed",False)
		handle_torso = sss.move("torso","home",False)
		handle_torso.wait()
		handle_sdh.wait()
		
		return 'success'		
		

class init_carry_box(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'])
		
	def execute(self, userdata):
		# command the operator
		sss.move("head", "back", False)
		SAY("Please wait a moment")
		
		guiding_client = rospy.ServiceProxy('/mdr_behaviors/haptic_guidance/start', std_srvs.srv.Empty)
		rospy.wait_for_service('/mdr_behaviors/haptic_guidance/start', 3)
		try:
			guiding_client()
			SAY("We can carry the box now!")
			return 'success'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'

class stop_carry_box(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'])
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self,userdata):
		guiding_client_pause = rospy.ServiceProxy('/mdr_behaviors/haptic_guidance/stop', std_srvs.srv.Empty)
		rospy.wait_for_service('/mdr_behaviors/haptic_guidance/stop', 3)
		sss.move("sdh","cylopen",True)
		rospy.sleep(1)
		
		try:
			guiding_client_pause()
			SAY("Thank you!")
			result = "success"
		except:
			print "Service call failed: %s"%e
			result = 'failed'
		sss.move("sdh","cylclosed")
		self.arm.set_named_target("folded")
		self.arm.go()
		sss.move("head", "front", False)
		return result
		
class carry_beer_box(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, outcomes=['failed', 'success'])
		self.userdata.commands_to_wait_for = ["stop_follow_me", "stop_guiding"]
		self.userdata.grasp_pose = ""
			
		with self:	
			
			smach.StateMachine.add('change_grammar_guiding', init_speech("guide.xml"), transitions={'success':'wait_for_box_in_hand', 																'failed':'failed'})
			
			smach.StateMachine.add('wait_for_box_in_hand', wait_for_box_in_hand(), transitions={'success':'init_carry_box', 																						'failed':'failed'})									
								
				# init the guiding behavior	
			smach.StateMachine.add('init_carry_box', init_carry_box(), transitions={'success':'announce_guiding','failed':'failed'})
				#say how to stop
			smach.StateMachine.add('announce_guiding', say_state("Please tell me to stop by saying stop carrying"), 
							transitions={'success':'wait_for_guiding_stop_command'})	

				# wait for the stop command	
			smach.StateMachine.add('wait_for_guiding_stop_command', wait_for_command(), 
									transitions={'success':'confirm_carry_box'})
		
			smach.StateMachine.add('confirm_carry_box', acknowledge_command(), 
						transitions={'yes':'stop_carry_box', 'no':'wait_for_guiding_stop_command'})
		
			smach.StateMachine.add('stop_carry_box', stop_carry_box(), 
						transitions={'success':'success', 'failed':'failed'})
								
		

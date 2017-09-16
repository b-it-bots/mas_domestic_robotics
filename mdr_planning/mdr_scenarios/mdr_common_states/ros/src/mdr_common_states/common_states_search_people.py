#!/usr/bin/python

######################### IMPORTS #########################

import roslib
roslib.load_manifest('mdr_common_states')
import rospy
import smach
import smach_ros
import tf
from math import *

import std_msgs.msg
import std_srvs.srv
import actionlib_msgs.msg

import mcr_perception_msgs.srv

from mdr_common_states.common_states_speech import *

from simple_script_server import *
sss = simple_script_server()


person_distance_threshold = 1.0
camera_height = 1.40


class activate_people_detection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'])

		self.pub_body_detection_event = rospy.Publisher('/mcr_perception/body_detection_3d/event_in', std_msgs.msg.String)

		self.activate_leg_detection_srv_name = '/mcr_perception/leg_detection/start'
		self.activate_leg_detection_srv = rospy.ServiceProxy(self.activate_leg_detection_srv_name, std_srvs.srv.Empty)

	def execute(self, userdata):
		sss.move("torso", "home")
		sss.move("head", "front")

		print "wait for service: ", self.activate_leg_detection_srv_name
		rospy.wait_for_service(self.activate_leg_detection_srv_name, 3)

		print "activate people detection"
		try:
			self.pub_body_detection_event.publish(std_msgs.msg.String('e_start'))
			self.activate_leg_detection_srv()
			rospy.sleep(1)
		except rospy.ServiceException,e:
			print "Service call failed: %s"%e
			return 'failed'
		return 'success'


class deactivate_people_detection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'])

		self.pub_body_detection_event = rospy.Publisher('/mcr_perception/body_detection_3d/event_in', std_msgs.msg.String)

		self.deactivate_leg_detection_srv_name = '/mcr_perception/leg_detection/stop'
		self.deactivate_leg_detection_srv = rospy.ServiceProxy(self.deactivate_leg_detection_srv_name, std_srvs.srv.Empty)
	def execute(self, userdata):
		print "wait for service: ", self.deactivate_leg_detection_srv_name
		rospy.wait_for_service(self.deactivate_leg_detection_srv_name, 3)

		print "DEactivate people detection"
		try:
			self.pub_body_detection_event.publish(std_msgs.msg.String('e_stop'))
			self.deactivate_leg_detection_srv()
		except rospy.ServiceException,e:
			print "Service call failed: %s"%e
			return 'failed'
		return 'success'


class activate_leg_detection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'])

		self.activate_leg_detection_srv_name = '/mcr_perception/leg_detection/start'
		self.activate_leg_detection_srv = rospy.ServiceProxy(self.activate_leg_detection_srv_name, std_srvs.srv.Empty)

	def execute(self, userdata):
		print "activate leg detection"

		rospy.wait_for_service(self.activate_leg_detection_srv_name, 3)
		try:
			self.activate_leg_detection_srv()
		except rospy.ServiceException,e:
			print "Service call failed: %s"%e
			return 'failed'
		return 'success'


class deactivate_leg_detection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'])

		self.deactivate_leg_detection_srv_name = '/mcr_perception/leg_detection/stop'
		self.deactivate_leg_detection_srv = rospy.ServiceProxy(self.deactivate_leg_detection_srv_name, std_srvs.srv.Empty)
	def execute(self, userdata):
		print "DEactivate leg detection"
		rospy.wait_for_service(self.deactivate_leg_detection_srv_name, 3)
		try:
			self.deactivate_leg_detection_srv()
		except rospy.ServiceException,e:
			print "Service call failed: %s"%e
			return 'failed'
		return 'success'


class approach_pose_selection(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['take_next_pose', 'continue_old_pose'], input_keys=['room_pose_to_approach'])

		# TODO: replace this using tf
		#self.get_current_pose = rospy.ServiceProxy('/base_controller/get_base_pose', mcr_navigation_msgs.srv.GetBasePose) 

	def execute(self, userdata):
		current_pose = None

		# if not pose is selected, take the first one
		if (len(userdata.room_pose_to_approach) == 0):
			print "room pose is emty -> take next pose"
			return 'take_next_pose'
		else:
			# TODO: replace this using tf
			#rospy.wait_for_service('/base_controller/get_base_pose', 3)
			try:
			    # TODO: replace this using tf
				#current_pose = self.get_current_pose()
				rospy.logerr("THIS NEEDS TO BE IMPLEMENTED, IT'S JUST A DUMMY CURRENTLY ! ! !")
			except rospy.ServiceException,e:
				print "Service call failed: %s"%e
				return 'failed'

			# get desired target position
			target_pose = rospy.get_param('/script_server/base/' + userdata.room_pose_to_approach)

			# get distance between target and current position
			eucl_dist = sqrt(pow(target_pose[0] - current_pose.pose.position.x, 2.0) + pow(target_pose[1] - current_pose.pose.position.y, 2.0))

			# if close to target, take the next pose, otherwise continue to approach the last target pose
			if(eucl_dist < 0.2):
				print "target pose reached (dist < 0.2) -> take next pose"
				return 'take_next_pose'
			else:
				print 'continue approaching old pose'
				print userdata.room_pose_to_approach
				return 'continue_old_pose'


class get_next_pose(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'empty_pose_list'], input_keys=['room_poses', 'room_pose_to_approach'], output_keys=['room_pose_to_approach', 'approach_state'])

	def execute(self, userdata):
		if(len(userdata.room_poses) > 0):
			userdata.room_pose_to_approach = userdata.room_poses.pop()
			userdata.approach_state = 0;
			print 'take next pose: ' + userdata.room_pose_to_approach
			return 'success'
		else:
			print 'all poses done'
			return 'empty_pose_list'


class check_if_persons_are_present(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['person_found', 'no_person_found', 'failed'], 
									input_keys=['visited_person_poses'],
									output_keys=['person_poses_to_approach', 'approach_state'])

	def execute(self, userdata):

		try:
			body_detections = rospy.wait_for_message('/mcr_perception/body_detection_3d/people_positions', mcr_perception_msgs.msg.PersonList, 1)
		except rospy.ROSException, e:
			print "timeout during wait for message: %s"%e
			return 'no_person_found'

		if len(body_detections.persons) > 0: 
			#check if person(s) already visited
			for found_person in body_detections.persons:
				for visited_person in userdata.visited_person_poses:
					global person_distance_threshold
					if sqrt(pow(visited_person[0] - found_person.pose.pose.position.x, 2.0) + pow(visited_person[1] - found_person.pose.pose.position.y, 2.0)) < person_distance_threshold:
						print "person alreay exists in visited list"
						return 'no_person_found'

			#stop base
			rospy.wait_for_service('base_controller/stop', 10)
			try:
				stop = rospy.ServiceProxy('base_controller/stop', Trigger)
				stop() 
			except rospy.ServiceException, e:
				error_message = "%s"%e
				rospy.logerr("calling <<%s>> service not successfull, error: %s", 'base_controller/stop', error_message)

			#write found person(s) to world model
			userdata.person_poses_to_approach = body_detections.persons
			userdata.approach_state = 0
			print "found %d persons(s)" %len(body_detections.persons)
			return 'person_found'
		else:
			print "no person found"
			return 'no_person_found'


class select_person_to_approach(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['all_persons_approached', 'person_pose_selected', 'person_already_visited'], 
					input_keys=['person_poses_to_approach', 'visited_person_poses'], 
					output_keys=['person_poses_to_approach', 'selected_person_pose', 'selected_person_safe_pose', 'visited_person_poses', 'selected_person_height'])

	def execute(self, userdata):
		if len(userdata.person_poses_to_approach) == 0: 
			print 'all persons position used'
			return 'all_persons_approached';
		else:
			print 'found personsssss %d' %len(userdata.person_poses_to_approach)
			person = userdata.person_poses_to_approach.pop()

			print 'person:'
			print person.pose.pose.position

			print "persons in visited lists: %d" %len(userdata.visited_person_poses)
			#check if person already visited
			for item in userdata.visited_person_poses:
				print 'item:'
				print item

				print 'dist:'
				print sqrt(pow(item[0] - person.pose.pose.position.x, 2.0) + pow(item[1] - person.pose.pose.position.y, 2.0))
				global person_distance_threshold
				if sqrt(pow(item[0] - person.pose.pose.position.x, 2.0) + pow(item[1] - person.pose.pose.position.y, 2.0)) < person_distance_threshold:
					print "person alreay exists in visited list"
					return 'person_already_visited'

			(roll, pitch, yaw) = tf.transformations.euler_from_quaternion([person.safe_pose.pose.orientation.x,
																		person.safe_pose.pose.orientation.y,
																		person.safe_pose.pose.orientation.z,
																		person.safe_pose.pose.orientation.w])


			userdata.selected_person_height = person.pose.pose.position.z + (person.height / 2)
			userdata.selected_person_pose = [person.pose.pose.position.x, person.pose.pose.position.y, yaw] 
			userdata.selected_person_safe_pose = [person.safe_pose.pose.position.x, person.safe_pose.pose.position.y, yaw] 

			print "person not yet in visited list"
			return 'person_pose_selected'


class store_person_position(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['stored'], input_keys=['selected_person_pose', 'visited_person_poses'], output_keys=['visited_person_poses'])

	def execute(self, userdata):
		userdata.visited_person_poses.append(userdata.selected_person_pose)
		print "person stored in vitied list"
		return 'stored'


class set_head_angle(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], input_keys=['selected_person_height', 'selected_person_pose'])
		# TODO: replace this using tf
		#self.get_current_pose = rospy.ServiceProxy('/base_controller/get_base_pose', mcr_navigation_msgs.srv.GetBasePose)

	def execute(self, userdata):
		current_pose = None

		# TODO: replace this using tf
		#rospy.wait_for_service('/base_controller/get_base_pose', 3)
		try:
       		# TODO: replace this using tf
			#current_pose = self.get_current_pose()
			rospy.logerr("THIS NEEDS TO BE IMPLEMENTED, IT'S JUST A DUMMY CURRENTLY ! ! !")
		except rospy.ServiceException,e:
			print "Service call failed: %s"%e
			return 'failed'

		#get distance between target and current position
		eucl_dist = sqrt(pow(userdata.selected_person_pose[0] - current_pose.pose.position.x, 2.0) + pow(userdata.selected_person_pose[1] - current_pose.pose.position.y, 2.0))

		#if(userdata.selected_person_height <= camera_height):
		#	angle = -math.pi
		#else:
		#	angle = -math.pi + atan( (userdata.selected_person_height - camera_height) / eucl_dist)

		#rospy.set_param('/script_server/head/dummy', [[angle]])

		#handle_head = sss.move("head", "dummy", False)
		#handle_head.wait()

		if userdata.selected_person_height <= 1.40:
			handle_head = sss.move("head", "front")
			handle_torso = sss.move("torso", "front")
		else:
			handle_head = sss.move("head", "front_face")


		SAY("Hey. I found you")


		#handle_head = sss.move("head", "front", False)
		#handle_head.wait()

		return 'success'


class check_if_person_is_in_front(smach.State):
	init_tfl = False
	transform_listener = None

	def __init__(self, distance_threshold = 1.0, angle_threshold = 0.7):
		smach.State.__init__(self, outcomes=['person_found', 'failed'])

		self.get_person_list = rospy.ServiceProxy('/mcr_perception/leg_detection/leg_positions', mcr_perception_msgs.srv.GetPersonList) 

		self.distance_threshold = distance_threshold
		self.angle_threshold = angle_threshold

	def execute(self, userdata):
		rospy.wait_for_service('/mcr_perception/leg_detection/leg_positions', 3)
		
		if check_if_person_is_in_front.init_tfl == False:
			check_if_person_is_in_front.transform_listener = tf.TransformListener()
			check_if_person_is_in_front.init_tfl = True
		
		while(True):
			try:
				person_list = self.get_person_list().person_list.persons
			except rospy.ServiceException,e:
				print "Service call failed: %s"%e
				return 'failed'

			for found_person in person_list	:
				try:
					check_if_person_is_in_front.transform_listener.waitForTransform(found_person.pose.header.frame_id, '/base_link', rospy.Time.now(), rospy.Duration(3.0))
					pose_transformed = check_if_person_is_in_front.transform_listener.transformPose('base_link', found_person.pose)
					
					if sqrt(pow(pose_transformed.pose.position.x, 2.0) + pow(pose_transformed.pose.position.y, 2.0)) < self.distance_threshold and fabs(atan(pose_transformed.pose.position.y / pose_transformed.pose.position.x)) < self.angle_threshold:
						return 'person_found'
				except:
					print "tf exception"

			rospy.sleep(0.5)

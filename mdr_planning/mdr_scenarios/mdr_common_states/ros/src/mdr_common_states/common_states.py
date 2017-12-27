#!/usr/bin/python

######################### IMPORTS #########################
import rospy
import smach
import smach_ros
import moveit_commander

from simple_script_server import *
sss = simple_script_server()

import std_srvs.srv
import actionlib_msgs.msg

import mcr_perception_msgs.srv

from mdr_common_states.common_states_face_recognition import *
from mdr_common_states.common_states_follow_me import *
from mdr_common_states.common_states_manipulation import *
from mdr_common_states.common_states_navigation import *
from mdr_common_states.common_states_object_perception import *
from mdr_common_states.common_states_search_people import *
from mdr_common_states.common_states_speech import *
from mdr_common_states.common_states_carry_box import *


######################### FUNCTION TO CHANGE THE COLOR OF THE COB LED'S #########################
COLOR_RED = ColorRGBA(1.0, 0.0, 0.0, 0.1)
COLOR_GREEN = ColorRGBA(0.0, 1.0, 0.0, 0.1)


def set_light_color(color):
	light_pub = rospy.Publisher('light_controller/command', ColorRGBA, latch=True)

	light_pub.publish(color)


class init_head_for_recognition(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed'])

	def execute(self, userdata):
		handle_head = sss.move("head","front_face")
		handle_head.wait()
		retval_list = []
		for retval in retval_list:
			if retval != 0:
				print 'Something failed:', retval_list
				return 'failed'
		return 'success'



class wait_for_start(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'pending'])

	def execute(self, userdata):
		print "Type -- S -- to start the scenario"
		ret = sss.wait_for_input()
		if ret == 'S':
			return 'success'
		else:
			print "Wrong input key"
			return 'pending'

class wait_for_door(smach.State):
	def __init__(self, timeout):
		smach.State.__init__(self, outcomes=['succeeded','timeout'])
                self.sub_door_status = rospy.Subscriber("/mcr_perception/door_status/door_status", std_msgs.msg.Bool, self.cb_door_status)
                self.status = None
                self.timeout = timeout

        def cb_door_status(self, msg):
            self.status = msg.data

	def execute(self, userdata):
            timeout = rospy.Duration.from_sec(self.timeout)
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < timeout:
                if self.status:
                    return 'succeeded'
                rate.sleep()
            return 'timeout'

class wait_for_qr(smach.State):
	def __init__(self, message, timeout):
		smach.State.__init__(self,
			outcomes=['succeeded', 'timeout'])
                self.message = message
                self.timeout = timeout
                self.sub_qr = rospy.Subscriber('/mcr_perception/qr_reader/output', std_msgs.msg.String, self.qr_cb)
                self.qr_message = None

        def qr_cb(self, txt):
            self.qr_message = txt.data

	def execute(self, userdata):
            self.qr_message = None
            timeout = rospy.Duration.from_sec(self.timeout)
            rate = rospy.Rate(10)
            start_time = rospy.Time.now()
            while (rospy.Time.now() - start_time) < timeout:
                if self.qr_message:
                    if self.qr_message.lower().find('continue') != -1:
                        return 'succeeded'
                    rospy.loginfo("QR message: %s" % self.qr_message)
                rate.sleep()
            return 'timeout'

class clear_costmaps(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'])

		self.clear_costmaps_srv_name = "/move_base/clear_costmaps"
		self.clear_costmaps_srv = rospy.ServiceProxy(self.clear_costmaps_srv_name, std_srvs.srv.Empty)

	def execute(self, userdata):
		rospy.wait_for_service(self.clear_costmaps_srv_name, 5)
		try:
			self.clear_costmaps_srv()
		except rospy.ServiceException, e:
			print("Service call to {0} failed: {1}".format(self.clear_costmaps_srv_name, e))
			return 'failed'
                return 'success'

class enter_arena(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'])

	def execute(self, userdata):

		door_client = rospy.ServiceProxy('/mcr_perception/door_status/door_status', mcr_perception_msgs.srv.GetDoorStatus) # TODO: topic-to-serice does not exist anymore
		rospy.wait_for_service('/mcr_perception/door_status/door_status',5) # todo error handling

		wait_for_open_door_phrase = "I am waiting for an open door"
		SAY(wait_for_open_door_phrase)

		# wait for open door
		door_open = False
		while not door_open:
			print door_open
			try:
				res = door_client()
			except rospy.ServiceException, e:
				print "Service call failed: %s"%e
			door_open = res.value

		enter_arena_phrase = "The door is open. I will enter the arena now"
		SAY(enter_arena_phrase)

		#sss.move("base","entrance_post_in",mode="linear")
		move_base_direct_brsu([0.2, 0.0, 0.0, 7])
		sss.move("base","entrance_post_in")
		#sss.move("torso","shake")

		return 'success'


############ counting and comparing states

class increment(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], input_keys=['number_to_increment', 'increment'], output_keys=['result'])

	def execute(self, userdata):
		userdata.result = userdata.number_to_increment + userdata.increment
		return 'success'

class compare(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['equal', 'not_equal'], input_keys=['value1', 'value2'])

	def execute(self, userdata):
		if userdata.value1 == userdata.value2:
			return 'equal'
		else:
			return 'not_equal'

class pop_item_from_list(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'no_item'],
									input_keys=['list_in'],
									output_keys=['popped_element_out'])

	def execute(self, userdata):
			if len(userdata.list_in) == 0:
				return 'no_item'
			else:
				userdata.popped_element_out = userdata.list_in.pop()
				return 'success'


class load_search_poses_in_room(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['success'],
			input_keys=['room_name_in', 'room_prefix_in', 'room_suffix_in'],
			output_keys=['room_poses_out'])

	def execute(self, userdata):
		room_poses_result =  []

		param_available = True
		i = 1
		while param_available:
			pose_string = userdata.room_prefix_in + userdata.room_name_in + str(i)
			i += 1
			param_available = rospy.has_param('/script_server/base/' + pose_string)
			print param_available
			if param_available:
				room_poses_result.append(pose_string)

		room_poses_result.reverse()
		print room_poses_result
		print pose_string
		userdata.room_poses_out = room_poses_result
		return 'success'


class is_array_empty(smach.State):
	def __init__(self):
		smach.State.__init__(self,
			outcomes=['yes', 'no'],
			input_keys=['array_in'])

	def execute(self, userdata):
		if userdata.array_in:
			return 'no'
		else:
			return 'yes'


class general_checklist(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'])
	def execute(self, userdata):
		raw_input("\nSTART DASHBOARD!\n")
		raw_input("\nSTART WINDOWS NODES!\n")
		raw_input("\nSPEAKER VOLUME?\n")
		raw_input("\nCHECK NODES WITH ROBOT_MONITOR\n")
		raw_input("\nPROTECT EMERGENCY STOP LASER RANGE\n")
		raw_input("\nCHECK THE LOCALIZATION (INITIAL POSE)\n")
		raw_input("\nDASHBOARD: MOVE TORSO AND HEAD\n")
		raw_input("\nDASHBOARD: ARM TO FOLDED\n")
		raw_input("\nMOVE HEAD AT FRONT_FACE\n")
		raw_input("\nTEST SPEECH RECOGNITION (JENNY YES)\n")
		raw_input("\nWAIT FOR: I'M, READY NOW!\n")
		return 'success'

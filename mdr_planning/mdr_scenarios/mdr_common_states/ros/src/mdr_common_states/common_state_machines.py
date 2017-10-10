#!/usr/bin/python

######################### IMPORTS #########################

import roslib
roslib.load_manifest('mdr_common_states')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

import std_srvs.srv
import std_msgs.msg
import actionlib_msgs.msg

from mdr_common_states.common_states import *
from mdr_common_states.common_states_speech import *
from mdr_common_states.common_states_search_people import *
from mdr_common_states.common_states_face_recognition import *
from mdr_common_states.common_states_navigation import *

''' 
	learn a person
	Learns a person and returns its name 
'''
class sm_learn_person(smach.StateMachine):
	def __init__(self, names_to_understand, grammar_to_load_after_finishing):	
		smach.StateMachine.__init__(self, outcomes=['success_learned_person', 'overall_failed'],
											output_keys=['sm_learn_person_person_name_out'])
		
		self.userdata.sm_learn_person_names_to_understand = names_to_understand 	
				
		with self:				
			smach.StateMachine.add('init_speech', init_speech("learn_person_top_level.xml"), transitions={'success':'explain_instructions', 'failed':'failed_recover_grammar'})			
					
			smach.StateMachine.add('explain_instructions', say_state('Please stand in front of me'), 
									transitions={'success':'is_person_in_front'})
			
			smach.StateMachine.add('is_person_in_front', is_person_in_front(), 
									transitions={'failed':'ask_user_for_name', 'success':'ask_user_for_name'})
			
			smach.StateMachine.add('ask_user_for_name', say_state("Please tell me your name"), 
									transitions={'success':'wait_for_name'})	
	
			smach.StateMachine.add('wait_for_name', wait_for_command(), 
									transitions={'success':'ack_name'},
									remapping={'commands_to_wait_for':'sm_learn_person_names_to_understand',
											'understood_command':'sm_learn_person_person_name_out'})
			
			smach.StateMachine.add('ack_name', acknowledge_command(), 
									transitions={'yes':'learn_face', 'no':'ask_user_for_name'},
									remapping={'understood_command':'sm_learn_person_person_name_out'})
			
			smach.StateMachine.add('learn_face', learn_face(),
									transitions={'success':'success_recover_grammar', 'failed':'learn_face'},
									remapping={'understood_command':'sm_learn_person_person_name_out'})			
												
			#If success or failure of the state machine we have to recover the previous grammar	
			smach.StateMachine.add('success_recover_grammar', init_speech(grammar_to_load_after_finishing), transitions={'success':'success_learned_person', 'failed':'overall_failed'})

			smach.StateMachine.add('failed_recover_grammar', init_speech(grammar_to_load_after_finishing), transitions={'success':'overall_failed', 'failed':'overall_failed'})


class load_search_poses_in_room_gpsr(smach.State):
	def __init__(self):
		smach.State.__init__(self, 
			outcomes=['success'], 
			input_keys=['room_name_in', 'room_prefix_in', 'room_suffix_in'],
			output_keys=['room_poses_out'])
		
	def execute(self, userdata):
		room_poses_result =  []
		speech_locations = rospy.get_param("/script_server/speech_places")	

		roomname = userdata.room_name_in
		
		if(roomname in speech_locations):
			param_available = True
			i = 1
			while param_available:
				pose_string = userdata.room_prefix_in + userdata.room_name_in + str(i)
				i += 1
				param_available = rospy.has_param('/script_server/base/' + pose_string)
				if param_available:
					room_poses_result.append(pose_string)
		
		print room_poses_result	
		userdata.room_poses_out = room_poses_result		
		return 'success'


#search for a specific object in predefined positions. returns the grasp and base pose
class sm_search_object(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, outcomes=['success_search_object', 'overall_failed'],
											output_keys=['sm_search_object_grasp_pose_out','sm_search_object_base_pose_out'],
											input_keys=['sm_search_object_room_to_search_in',
														'sm_search_object_room_prefix_in',
														'sm_search_object_room_suffix_in',
														 'sm_search_object_object_to_search_in'])
		
		self.userdata.sm_search_object_room_poses = []
		
		with self:	
			
			smach.StateMachine.add('select_room_poses', load_search_poses_in_room_gpsr(),
								transitions={'success':'select_pose'},
								remapping={'room_name_in':'sm_search_object_room_to_search_in',
											'room_prefix_in' : 'sm_search_object_room_prefix_in',
											'room_suffix_in' : 'sm_search_object_room_suffix_in',
											'room_poses_out':'sm_search_object_room_poses'})						
		
			smach.StateMachine.add('select_pose', pop_item_from_list(),
								transitions={'success':'approach_pose',
											'no_item':'overall_failed'},
								remapping={'popped_element_out':'sm_search_object_base_pose_out',
											'list_in':'sm_search_object_room_poses'})
								
			smach.StateMachine.add('approach_pose', approach_pose(),
								transitions={'success':'search_for_object',
											'failed':'select_pose'},
								remapping={'pose':'sm_search_object_base_pose_out'})
								
			smach.StateMachine.add('search_for_object', find_object_moped(),
								transitions={'success':'success_search_object',
											'failed':'select_pose'},
								remapping={'grasp_position': 'sm_search_object_grasp_pose_out',
											'object_name': 'sm_search_object_object_to_search_in'})


'''
Searches a person with the laser based people detection by approaching poses passed in the userdata
The found person positions are then returned or a fail if no persons were found
'''		
class sm_search_a_person(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, outcomes=['success_search_person', 'overall_failed'],
											output_keys=['room_poses'],
											input_keys=['room_poses'])
		
		#sm.userdata.room_poses = [] input
		self.userdata.room_name = ""
		self.userdata.room_pose_to_approach = ""
		self.userdata.person_poses_to_approach = []
		self.userdata.selected_person_pose = ""
		self.userdata.selected_person_height = 0.0
		self.userdata.selected_person_safe_pose = ""
		self.userdata.timer = 0
		self.userdata.approach_state = 0
		self.userdata.visited_person_poses = [];
		
		with self:	
			smach.StateMachine.add('activate_people_detection', activate_people_detection(), 
								transitions={'success':'approach_pose_selection', 'failed':'activate_people_detection'})

			smach.StateMachine.add('approach_pose_selection', approach_pose_selection(), 
									transitions={'take_next_pose':'get_next_pose', 'continue_old_pose':'approach_pose_without_retry_non_blocking'})

			smach.StateMachine.add('get_next_pose', get_next_pose(), 
									transitions={'success':'approach_pose_without_retry_non_blocking', 
									'empty_pose_list':'success_search_person'})

			smach.StateMachine.add('approach_pose_without_retry_non_blocking', approach_pose_without_retry_non_blocking(), 
									transitions={'pose_reached':'get_next_pose', 
									'could_not_reach_pose':'get_next_pose',
									'base_is_moving':'check_if_persons_are_present', 
									'failed':'get_next_pose'},
									remapping={'target_pose':'room_pose_to_approach'})
		

			smach.StateMachine.add('check_if_persons_are_present', check_if_persons_are_present(), 
									transitions={'no_person_found':'approach_pose_without_retry_non_blocking', 
									'person_found':'deactivate_people_detection', 'failed':'approach_pose_without_retry_non_blocking'})

			smach.StateMachine.add('deactivate_people_detection', deactivate_people_detection(), 
									transitions={'success':'select_person_to_approach', 'failed':'deactivate_people_detection'})

			smach.StateMachine.add('select_person_to_approach', select_person_to_approach(), 
									transitions={'person_pose_selected':'approach_person', 'all_persons_approached':'activate_people_detection', 
									'person_already_visited':'select_person_to_approach'})

			smach.StateMachine.add('approach_person',  approach_pose_without_retry(), 
									transitions={'success':'store_person_position',
									 'failed':'select_person_to_approach'},
									remapping={'pose':'selected_person_safe_pose'})

			smach.StateMachine.add('store_person_position', store_person_position(), 
									transitions={'stored':'set_head_angle_according_to_person_height'})

			smach.StateMachine.add('set_head_angle_according_to_person_height', set_head_angle(), transitions={'success':'success_search_person'})
			
#search for a person in front of jenny 
class is_person_in_front(smach.StateMachine):
	def __init__(self):	
		smach.StateMachine.__init__(self, outcomes=['failed', 'success'])
			
		with self:	
			
			smach.StateMachine.add('activate_leg_detection', activate_leg_detection(),
								transitions={'success':'check_if_person_is_in_front', 'failed':'failed'})						
		
			smach.StateMachine.add('check_if_person_is_in_front', check_if_person_is_in_front(),
								transitions={'person_found':'deactivate_leg_detection', 'failed':'failed'})	
								
			smach.StateMachine.add('deactivate_leg_detection', deactivate_leg_detection(),
								transitions={'success':'success', 'failed':'failed'})	
								
	


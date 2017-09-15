#!/usr/bin/python

import rospy
import smach
import smach_ros
import moveit_commander

import mcr_speech_msgs.msg
import mcr_perception_msgs.msg

from mdr_common_states.common_states import *
from mdr_common_states.common_states_speech import *

import mdr_common_states as cs

INTRODUCE = 'introduce'
LEARN_PERSON = 'learn_person'
FIND_PERSON = 'find_person'
IDENTIFY_PERSON = 'identify_person'
FOLLOW = 'follow_person'
GUIDE = 'guide'
MOVE_TO = 'move_to'
DETECT_OBJECT = 'detect_object'
POINT_TO_OBJECT = 'point_to_object'
GRASP = 'grasp'
CLEAN_TABLE = 'clean_table'
EXIT = 'exit'
PENDING = 'pending'
WEIGHT = 'weight'
RELEASE = 'release'
HANDOVER = 'handover'
CATEGORIZE_OBJECTS = 'categorize_objects'
CARRY_BOX = 'carry_box'

GPSR_PREFIX = 'gpsr_'

IT = "it"


class load_faces(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'])
		self.load_person_face = rospy.ServiceProxy('/mcr_perception/face_recognition/load_person_face', mcr_perception_msgs.srv.SetFaceName)

	def execute(self, userdata):
		#rospy.wait_for_service('/mcr_perception/face_recognition/load_person_face', 3)
		cs.common_states.set_light_color(cs.common_states.COLOR_RED)
		try:
			#self.load_person_face("nico")
			#self.load_person_face("fred")
			#self.load_person_face("jan")
			#self.load_person_face("sven")
			#self.load_person_face("azden")
			#self.load_person_face("rhama")
			return 'success'
		except rospy.ServiceException, e:
			print "Service call failed: %s"%e
			return 'failed'
		
		
class move_actuators_to_home_poses(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		sss.move("head","front_face")
		sss.move("torso","home")	
		sss.move("tray","down")

		return 'success'


class Speech_keyword():
	def __init__(self, keyword="", confidence=0.0):
		self.keyword = keyword
		self.confidence = confidence

# Command to execute contains action, object and location
class Speech_command():
	
	def __init__(self, the_action="", the_object="", the_location =""):
		self.speech_action = Speech_keyword(keyword=the_action)
		self.speech_object = Speech_keyword(keyword=the_object)
		self.speech_location = Speech_keyword(keyword=the_location)
	
		self.actions_with_object = [GRASP, CLEAN_TABLE, POINT_TO_OBJECT, DETECT_OBJECT]
		self.actions_with_location = [FIND_PERSON, DETECT_OBJECT, GRASP, CLEAN_TABLE, POINT_TO_OBJECT, HANDOVER, CATEGORIZE_OBJECTS]
	
	# create a speakable string representation	
	def createPhrase(self):
		phrase = self.speech_action.keyword
		#actions with object get it appended
		if self.speech_object.keyword != "" and self.speech_action.keyword in self.actions_with_object:
			#if Keyword is if, the "the" is obsolete
			if self.speech_object.keyword != IT:
				phrase = phrase + " the "
			phrase = phrase + self.speech_object.keyword
		if self.speech_location.keyword != "" and self.speech_action.keyword in self.actions_with_location:
			phrase = phrase + " in the " + self.speech_location.keyword
		if self.speech_location.keyword != "" and self.speech_action.keyword == MOVE_TO:
			phrase = phrase + " the " + self.speech_location.keyword
		return phrase


class detected_object():
	def __init__(self, new_name = "", new_object_grasp_position = [], new_object_grasp_base_position = ""):
		self.object_name = new_name
		self.object_grasp_position = new_object_grasp_position
		self.object_grasp_base_position = new_object_grasp_base_position
	
	
class Robot_state():
	
	def __init__(self):
		self.actual_location = ""
		self.object_in_hand = ""
		self.found_objects = []
	
	def grasped_object(self, object_name):
		#delete object from found ones
		for some_object in self.found_objects:
			if(some_object.object_name == object_name):
				self.found_objects.remove(some_object)
		self.object_in_hand = object_name
		
	def moved(self, pose):
		self.actual_location = pose
		
	def found_object(self, new_base_position, new_grasp_position, new_object_name, pose):		
		new_object = detected_object(new_name = new_object_name, new_object_grasp_position = new_grasp_position, new_object_grasp_base_position = new_base_position)
		self.found_objects.append(new_object)
		self.actual_location = pose
		
		
	def __str__(self):
		#print self.found_objects
		for item in self.found_objects:
			print "item_name: " + item.object_name
			print "bPose: " + item.object_grasp_base_position

		return "location: " + self.actual_location + " \nobject_in_hand: " + self.object_in_hand
		
	def handed_object_over(self):
		print "TODO"

class update_robot_state(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], 
						output_keys=['robot_state_out'],
						input_keys = ['robot_state_in', 'robot_position_in', 'object_name_in', 'executed_action_in', 'grasp_pose_in'])
	
	def execute(self, userdata):
		new_robot_state = Robot_state()
		new_robot_state.actual_location = userdata.robot_state_in.actual_location
		new_robot_state.object_in_hand = userdata.robot_state_in.object_in_hand
		new_robot_state.found_objects = userdata.robot_state_in.found_objects
				
		if(userdata.executed_action_in == MOVE_TO):
			new_robot_state.moved(userdata.robot_position_in)
		if(userdata.executed_action_in == GRASP):
			new_robot_state.grasped_object(userdata.object_name_in)
		if(userdata.executed_action_in == DETECT_OBJECT):
			new_robot_state.found_object(new_object_name = userdata.object_name_in, new_base_position = userdata.robot_position_in, new_grasp_position = userdata.grasp_pose_in, pose = userdata.robot_position_in)
			
		userdata.robot_state_out = new_robot_state
		return 'success'

class get_robot_state(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], 
						output_keys=['actual_location_out', 'object_in_hand_out', 'found_objects_out'],
						input_keys = ['robot_state_in'])
	
	def execute(self, userdata):
		userdata.actual_location_out = userdata.robot_state_in.actual_location
		userdata.object_in_hand_out = userdata.robot_state_in.object_in_hand
		userdata.found_objects_out = userdata.robot_state_in.found_objects
		
		return 'success'



# replaces a sequence in a given string with a given char set
class filter_from_string(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], input_keys=['string_to_filter_in', 'sequence_to_remove_in','replacement_in' ], output_keys=['result_string_out'])

	def execute(self, userdata):
		userdata.result_string_out = userdata.string_to_filter_in.replace(userdata.sequence_to_remove_in, userdata.replacement_in)
		return 'success'


class introduce(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'])
		self.arm = moveit_commander.MoveGroupCommander('arm')

	def execute(self, userdata):
		handle_base = sss.move("base", "living_room", False)
		handle_base.wait()

		SAY("Hello ladies and gentlemen. My name is Jenny. I am a Care O bot 3 robot.")
		sss.sleep(1)	
		SAY("I live at the bonn rhein sieg university in sankt augustin. I have my own appartment in the RoboCup lab. And I am a member of the b i t bots team.")
		sss.sleep(1)
		SAY("I am designed to be an autonomous domestic service robot. This means that I can help you with your household chores.")

		handle_tray = sss.move("tray","up",False)
                sss.sleep(2)
                SAY("With my tray I can carry multiple objects at the same time and hand them over to my guests.")
                handle_tray.wait()

		self.arm.set_named_target("look_at_table")
		self.arm.go(wait=False)
		SAY("I am equipped with a 7 degree of freedom light weight arm and a three finger gripper.")

		handle_base = sss.move("base", "second_intro", False)
		SAY("My base is omnidirectional which allows me to move forwards, backwards, sideways and even turn at the same time.")
		handle_base.wait()

		handle_head = sss.move("head", "back", False)
		SAY("I can see with my color cameras and 3 d sensor. I can use them to detect objects and people.")
		handle_head = sss.move("head", "front", False)
		handle_head.wait()
		
		#self.arm.set_named_target("overtray_top")
		#self.arm.go()

		sss.move("gripper","cylopen")
		SAY("I'm able to reach positions on my front and backside with my arm. If you need a hand carrying things, I can lift up to 7 kilos.")

		handle_sdh = sss.move("gripper","cylclosed",False)
		handle_sdh.wait()

		self.arm.set_named_target("folded")
		self.arm.go()

		handle_tray = sss.move("tray","down",False)
		handle_tray.wait()

		SAY("Thank you for your attention. I hope you enjoy the rest of the show.")

		handle_torso = sss.move("torso","nod",False)
		handle_torso.wait()
		
		#SAY("Guys, do you need anything now or shall I return to my room?")
		
		sss.sleep(2)

		return 'success'


# takes any phrase and returns success if something understood. all understood keywords are in the userdata
class wait_for_arbitrary_phrase(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success','not_understood'], 
									output_keys=['keyword_list_out', 'confidence_list_out'])

		self.get_last_recognized_speech_srv_name = '/mcr_speech_recognition/get_last_recognized_speech'
		self.get_last_recognized_speech = rospy.ServiceProxy(self.get_last_recognized_speech_srv_name, mcr_speech_msgs.srv.GetRecognizedSpeech) # TODO this was a topic from topic-to-service. need to solve this
	
	def execute(self, userdata):
		# wait for the command
		cs.common_states.set_light_color(cs.common_states.COLOR_GREEN)
		rospy.wait_for_service(self.get_last_recognized_speech_srv_name, 3) # TODO this was a topic from topic-to-service. need to solve this
		res = self.get_last_recognized_speech()
		
		if res.keyword.strip() != "no_speech" and res.keyword.strip() != "not_understood":
			userdata.keyword_list_out = res.keyword_list
			userdata.confidence_list_out = list(res.confidence_list)
			cs.common_states.set_light_color(cs.common_states.COLOR_RED)
			return 'success'
		else:
			rospy.sleep(0.2)
			return 'not_understood'

# separates the understood phras into command / action / location 
class decompose_speech_phrase(smach.State):

	def __init__(self):
		smach.State.__init__(self, outcomes=['success','not_understood'], 
									input_keys=['keyword_list_in','confidence_list_in','previous_understood_action_list_in',
												'current_room_in'],
									output_keys=['understood_phrase_out', 'action_list_out']
									)
	
	def execute(self, userdata):
		speech_objects = rospy.get_param("/script_server/speech_objects")
		speech_objects.append('it') #count "it" as object	
		
		speech_locations = rospy.get_param("/script_server/speech_places")	
		#speech_locations = speech_locations + rospy.get_param("/script_server/speech_grasp_locations")	
			
		speech_actions = rospy.get_param("/script_server/speech_actions")
			
		words = userdata.keyword_list_in
		confidences = userdata.confidence_list_in
		
		print words
		
		#store the commands in
		speech_commands  = []
		#awful decomposition by many if's
		# As long as there are words available:
		# check if first word is an action
		# if yes, delete and check next word for object/location
		# if yes do this again for appropriate missing part
		while(words):
			c = Speech_command()
			
			#take first word (is always action and erase)
			if words[0] in speech_actions:
				c.speech_action = Speech_keyword(words[0], confidences[0])
				del words[0]
				del confidences[0]
				#if more words available and its a location, store it		
				if words and words[0] in speech_locations:
					c.speech_location = Speech_keyword(words[0], confidences[0])
					del words[0]						
					del confidences[0]
					#third may be a object -> store
					if words and words[0] in speech_objects:
						c.speech_object = Speech_keyword(words[0], confidences[0])
						del words[0]
						del confidences[0]
				#otherwise it might be object					
				elif words and words[0] in speech_objects:
					c.speech_object = Speech_keyword(words[0], confidences[0])
					del words[0]
					del confidences[0]
					#if more words available and its a location, store it		
					if words: 
						if words[0] in speech_locations:
							c.speech_location = Speech_keyword(words[0], confidences[0])
							del words[0]	
							del confidences[0]
				#Add the new command									
				speech_commands.append(c)
			else:
				return "not_understood"
					
		#check with confidence if previous understood or current shall be used
#		for i in range(len(userdata.previous_understood_action_list_in)):
#			# if actions differ, check the confidence
#			if userdata.previous_understood_action_list_in[i].speech_action.keyword != speech_commands[i].speech_action.keyword:
				# exchange with previous, if previous confidence was higher or confidence is the same (because previous was def. not understood)
#				if (userdata.previous_understood_action_list_in[i].speech_action.confidence - speech_commands[i].speech_action.confidence) > 0.001 :									
#					speech_commands[i].speech_action = userdata.previous_understood_action_list_in[i].speech_action
#					speech_commands[i].speech_location = userdata.previous_understood_action_list_in[i].speech_location
#					speech_commands[i].speech_object = userdata.previous_understood_action_list_in[i].speech_object
			#IF OBJECTS AND SO ARE THERE, DONT EXCHANGE THW WHOLE ACTION
			#actions are the same, check location and 
			#else:
			#	if userdata.previous_understood_action_list_in[i].speech_object.conidence > speech_commands[i].speech_object.confidence:
			#		speech_commands[i].speech_object = userdata.previous_understood_action_list_in[i].speech_object
			#	if userdata.previous_understood_action_list_in[i].speech_location.conidence > speech_commands[i].speech_location.confidence:
			#		speech_commands[i].speech_location = userdata.previous_understood_action_list_in[i].speech_location
		
		confirmation_phrase = ""
		for action in speech_commands:
			confirmation_phrase = confirmation_phrase + ", " + action.createPhrase()
											
		'''Many commands don't contain a location if e.g. the current room is Implicitly mentioned. For execution
		this is important -> add current room resp. last annouced room. Same for object
		This also handles phrases like "find the chips, grasp it" whre it implicitly referes to prev. item '''
		current_pose = userdata.current_room_in
		current_object = ""
		for action in speech_commands:
			if action.speech_location.keyword == "":
				action.speech_location.keyword = current_pose
				action.speech_location.confidence = 0.0
			else:
				current_pose = action.speech_location.keyword
			if action.speech_object.keyword == "" or action.speech_object.keyword == IT:
				action.speech_object.keyword = current_object
				action.speech_object.confidence = 0.0
			else:
				current_object = action.speech_object.keyword
		
		#Reverse actionlist since its used as a stack
		speech_commands.reverse()
		userdata.action_list_out = speech_commands
				
		userdata.understood_phrase_out = confirmation_phrase
		
		return 'success'


class execute_commands(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['success', 'failed',PENDING, INTRODUCE, LEARN_PERSON, FIND_PERSON,
											 IDENTIFY_PERSON, FOLLOW, GUIDE, MOVE_TO, DETECT_OBJECT,
											 GRASP, CLEAN_TABLE, POINT_TO_OBJECT, EXIT, WEIGHT, RELEASE, HANDOVER, CATEGORIZE_OBJECTS, CARRY_BOX],
									input_keys=['actions_to_execute_in', 'robot_state_in', 'already_executed_actions_in'],
									output_keys=['actions_left_out', 'pose_out', 'object_out', 'already_executed_actions_out', 'current_action_out'])
	
	def execute(self, userdata):
		#if all actions done return success
		if(not userdata.actions_to_execute_in):
			return 'success'
		
		action_list = userdata.actions_to_execute_in	
					
		#pop current action from the list and execute
		action_to_execute = action_list.pop()
		
		#Store the  next action / transition
		result = ''
		#announce action and  decompose or return appropriate transition
		if action_to_execute.speech_action.keyword == INTRODUCE:
			# Move exactly to appropriate location if not already there
			if action_to_execute.speech_location.keyword != userdata.robot_state_in.actual_location:	
				action_list.append(action_to_execute)
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, action_to_execute.speech_location.keyword))
				result = PENDING
			else:
				SAY("Please let me introduce myself")
				result = INTRODUCE
		elif action_to_execute.speech_action.keyword == LEARN_PERSON:
			# Move exactly to appropriate location if not already there
			if action_to_execute.speech_location.keyword != userdata.robot_state_in.actual_location:	
				action_list.append(action_to_execute)
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, action_to_execute.speech_location.keyword))
				result = PENDING
			else:
				SAY("I'll learn a person now.")
				result = LEARN_PERSON
		elif action_to_execute.speech_action.keyword == FIND_PERSON:
			# Move exactly to appropriate location if not already there
			if action_to_execute.speech_location.keyword != userdata.robot_state_in.actual_location:	
				action_list.append(action_to_execute)
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, action_to_execute.speech_location.keyword))
				result = PENDING
			else:
				SAY("I'm gonna find a person now.")
				result = FIND_PERSON	
		elif action_to_execute.speech_action.keyword == IDENTIFY_PERSON:
			# Move exactly to appropriate location if not already there
			if action_to_execute.speech_location.keyword != userdata.robot_state_in.actual_location:	
				action_list.append(action_to_execute)
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, action_to_execute.speech_location.keyword))
				result = PENDING
			else:
				SAY("I will identify a person now. Please come close.")
				result = IDENTIFY_PERSON	
		elif action_to_execute.speech_action.keyword == FOLLOW:
				# Move exactly to appropriate location if not already there
			if action_to_execute.speech_location.keyword != userdata.robot_state_in.actual_location:	
				action_list.append(action_to_execute)
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, action_to_execute.speech_location.keyword))
				result = PENDING
			else:
				result = FOLLOW		
		elif action_to_execute.speech_action.keyword == GUIDE:
			result = GUIDE		
		elif action_to_execute.speech_action.keyword == MOVE_TO:
			SAY("I'm moving to the " + action_to_execute.speech_location.keyword.replace(GPSR_PREFIX, ''))
			result =  MOVE_TO		
		elif action_to_execute.speech_action.keyword == DETECT_OBJECT:
			# check if we are in the same room (after find person we are not exactly in the room pose but person might be in front)
			if action_to_execute.speech_location.keyword not in userdata.robot_state_in.actual_location:	
				action_list.append(action_to_execute)
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, action_to_execute.speech_location.keyword))
				result = PENDING
			else:
				SAY("I'll search the " + action_to_execute.speech_object.keyword)
				result =  DETECT_OBJECT		
		elif action_to_execute.speech_action.keyword == POINT_TO_OBJECT:
			# If object detected, grasp, otherwise find object!		
			is_object_found = False
			# check if we know the object pose already
			for item in userdata.robot_state_in.found_objects:
				#if the one request is there, it is known
				if item.object_name == action_to_execute.speech_object.keyword:
					is_object_found = True
			
			#if location of object is different than ours, put grasp action on the stack with location equal to the object location
			if (is_object_found):# and item.object_grasp_base_position != userdata.robot_state_in.actual_location):
				action_list.append(Speech_command(POINT_TO_OBJECT, action_to_execute.speech_object.keyword, item.object_grasp_base_position))                                                                                                                                                              
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, item.object_grasp_base_position))         
				result = PENDING
			else:
				SAY("I'll show you the " + action_to_execute.speech_object.keyword + " from the " + action_to_execute.speech_location.keyword.replace(GPSR_PREFIX, ''))			
				result =  POINT_TO_OBJECT	
		elif action_to_execute.speech_action.keyword == GRASP:
			# If object detected, grasp, otherwise find object!		
			is_object_found = False
			# check if we know the object pose already
			for item in userdata.robot_state_in.found_objects:
				#if the one request is there, it is known
				if item.object_name == action_to_execute.speech_object.keyword:
					is_object_found = True
				
			#if location of object is different than ours, put grasp action on the stack with location equal to the object location
			if(is_object_found and item.object_grasp_base_position != userdata.robot_state_in.actual_location):
				action_list.append(Speech_command(GRASP, action_to_execute.speech_object.keyword, item.object_grasp_base_position))																					
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, item.object_grasp_base_position))																	
				print "HAVE TO MOVE " + item.object_grasp_base_position
				result = PENDING
			else:
				SAY("I'll grasp the " + action_to_execute.speech_object.keyword + " from the " + action_to_execute.speech_location.keyword.replace(GPSR_PREFIX, ''))			
				result =  GRASP		
		elif action_to_execute.speech_action.keyword == CLEAN_TABLE:
			# If sponge detected, clean, otherwise find object!		
			is_object_found = False
			# check if we know the object pose already
			for item in userdata.robot_state_in.found_objects:
				#if the one request is there, it is known
				if item.object_name == action_to_execute.speech_object.keyword:
					is_object_found = True
				
			#if location of object is different than ours, put grasp action on the stack with location equal to the object location
			if(is_object_found and item.object_grasp_base_position != userdata.robot_state_in.actual_location):
				action_list.append(Speech_command(CLEAN_TABLE, action_to_execute.speech_object.keyword, item.object_grasp_base_position))																					
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, item.object_grasp_base_position))																	
				print "HAVE TO MOVE " + item.object_grasp_base_position
				result = PENDING
			else:
				SAY("I'll clean the table in the " + action_to_execute.speech_location.keyword.replace(GPSR_PREFIX, ''))			
				result = CLEAN_TABLE	
			
		elif action_to_execute.speech_action.keyword == EXIT:
			SAY("I'll leave the arena now.")
			result = EXIT
		elif action_to_execute.speech_action.keyword == WEIGHT:
			SAY("I'll weigh a bottle now.")
			result = WEIGHT
		elif action_to_execute.speech_action.keyword == CATEGORIZE_OBJECTS:
			SAY("I will categorize the objects now.")
			result = CATEGORIZE_OBJECTS
		elif action_to_execute.speech_action.keyword == RELEASE:
			result = RELEASE
		elif action_to_execute.speech_action.keyword == CARRY_BOX:
			result = CARRY_BOX
		elif action_to_execute.speech_action.keyword == HANDOVER:
			# check if we are in the same room
			if action_to_execute.speech_location.keyword not in userdata.robot_state_in.actual_location:	
				action_list.append(action_to_execute)
				action_list.append(Speech_command(MOVE_TO, action_to_execute.speech_object.keyword, action_to_execute.speech_location.keyword))
				result = PENDING
			else:
				result = HANDOVER
		
		
		else:
			result = 'failed'
			
		# store pose and object in userdata
		userdata.pose_out = action_to_execute.speech_location.keyword
		userdata.object_out = action_to_execute.speech_object.keyword
		
		#return open actions for next execution (either all if a pseudo move to was used, or all minus the next executed one)
		userdata.actions_left_out = action_list
		
		#append next action to the already executed
		tmp = userdata.already_executed_actions_in
		tmp.append(action_to_execute)
		userdata.already_executed_actions_out = tmp
		
		userdata.current_action_out = result
		return result


class get_room_poses(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], 
					input_keys=['room_name', 'room_poses'],
					output_keys=['room_poses'])
									
	def execute(self, userdata):
		userdata.room_poses = []
		for i in range(1,5): #for poses living_room1 - living_room4
			pose = userdata.room_name + str(i)
			print pose
			userdata.room_poses.append(pose)
		
		sss.move("head", "front")
		return 'success'	
		
class move_to_exit(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['success'])

	def execute(self, userdata):
		SAY("Goodbye")
		sss.move("base","exit")
		return 'success'


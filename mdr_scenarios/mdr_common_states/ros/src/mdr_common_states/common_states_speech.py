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

import mcr_speech_msgs.srv

from mdr_common_states.common_states import *

import mdr_common_states as cs

######################### GLOBAL SAY TO AVOID SCRIPT SERVER #########################
def SAY(text, blocking=True, timeout=60):
		
	speak_pub = rospy.Publisher('/say', std_msgs.msg.String, latch=True)

	speak_pub.publish(std_msgs.msg.String(text))
	
	time_start = rospy.Time.now()
	if(blocking == True):
		while(True):
			try:
				event_msg = rospy.wait_for_message('/mcr_speech_synthesis/event_out', std_msgs.msg.String, timeout=timeout)

				if((event_msg.data == 'e_done') or ((rospy.Time.now() - time_start) > timeout)):
					break;
			except rospy.ROSException, e:
				print "timeout during wait for message: %s"%e
		
	clear_last_command_name = '/mcr_speech_recognition/clear_last_recognized_speech'
	clear_last_command_proxy = rospy.ServiceProxy(clear_last_command_name, std_srvs.srv.Empty)
	rospy.wait_for_service(clear_last_command_name, 3)
	clear_last_command_proxy()


class init_speech(smach.State):
	grammar = 'follow_me.xml'
	def __init__(self, grammarfile):
		smach.State.__init__(self, outcomes=['success', 'failed'])
		self.grammar = grammarfile

		self.change_grammar_srv_name = '/mcr_speech_recognition/change_grammar'
		self.change_grammar = rospy.ServiceProxy(self.change_grammar_srv_name, mcr_speech_msgs.srv.ChangeGrammar)

	def execute(self, userdata):

		rospy.wait_for_service(self.change_grammar_srv_name, 5)
		try:
			self.change_grammar(self.grammar)
			return 'success'
		except rospy.ServiceException, e:
			print "Service called failed: %s"%e
			return 'failed'

class wait_for_command(smach.State):
	last_command = ""

	def __init__(self):
		smach.State.__init__(self, outcomes=['success'], 
									output_keys=['understood_command'], 
									input_keys=['commands_to_wait_for'])


		self.get_last_recognized_speech_srv_name = '/mcr_speech_recognition/get_last_recognized_speech'
		self.get_last_recognized_speech = rospy.ServiceProxy(self.get_last_recognized_speech_srv_name, mcr_speech_msgs.srv.GetRecognizedSpeech)

	def execute(self, userdata):
		# wait for the command
		cs.common_states.set_light_color(cs.common_states.COLOR_GREEN)
		rospy.wait_for_service(self.get_last_recognized_speech_srv_name, 3)
		
		while(True):
			res = self.get_last_recognized_speech()
			self.last_command = res.keyword
			if self.last_command in userdata.commands_to_wait_for:
				break
			else:
				rospy.sleep(0.2)
				
		print self.last_command

		userdata.understood_command = self.last_command 
		cs.common_states.set_light_color(cs.common_states.COLOR_RED)	
		return 'success'



class acknowledge_command(smach.State):
	lastCommand = ""

	def __init__(self):
		smach.State.__init__(self, outcomes=['yes','no'], input_keys=['understood_command'])

		self.get_last_recognized_speech_srv_name = '/mcr_speech_recognition/get_last_recognized_speech'
		self.get_last_recognized_speech_srv = rospy.ServiceProxy(self.get_last_recognized_speech_srv_name, mcr_speech_msgs.srv.GetRecognizedSpeech)

		self.clear_last_command_proxy_srv_name = '/mcr_speech_recognition/clear_last_recognized_speech'
		self.clear_last_command_proxy = rospy.ServiceProxy(self.clear_last_command_proxy_srv_name, std_srvs.srv.Empty)

	def execute(self, userdata):
		rospy.wait_for_service(self.get_last_recognized_speech_srv_name, 3)
		rospy.sleep(0.2)
	
		rospy.wait_for_service(self.clear_last_command_proxy_srv_name, 3)
		

		SAY('Did you say: ' + userdata.understood_command + ', is this correct?')
		print userdata.understood_command
		rospy.sleep(0.2)
		self.clear_last_command_proxy()
		# wait for the command
		cs.common_states.set_light_color(cs.common_states.COLOR_GREEN)

		while(True):
			res = self.get_last_recognized_speech_srv()
			if res.keyword == 'yes' or res.keyword == 'no':
				break
			else:
				rospy.sleep(0.2)

		print "Last recognized commmand (ACK) ", res.keyword
		if res.keyword == 'yes':
			cs.common_states.set_light_color(cs.common_states.COLOR_RED)
			return 'yes'
		elif res.keyword == 'no':
			cs.common_states.set_light_color(cs.common_states.COLOR_RED)
			return 'no'	


#loads only the Ack grammar
class acknowledge_command_with_loading_grammar(smach.State):
	lastCommand = ""

	def __init__(self):
		smach.State.__init__(self, outcomes=['yes','no'], input_keys=['understood_command'])

		self.get_last_recognized_speech_srv_name = '/mcr_speech_recognition/get_last_recognized_speech'
		self.get_last_recognized_speech_srv = rospy.ServiceProxy(self.get_last_recognized_speech_srv_name, mcr_speech_msgs.srv.GetRecognizedSpeech)

		self.clear_last_command_proxy_srv_name = '/mcr_speech_recognition/clear_last_recognized_speech'
		self.clear_last_command_proxy = rospy.ServiceProxy(self.clear_last_command_proxy_srv_name, std_srvs.srv.Empty)

		self.change_grammar_srv_name = '/mcr_speech_recognition/change_grammar'
		self.change_grammar = rospy.ServiceProxy(self.change_grammar_srv_name, mcr_speech_msgs.srv.ChangeGrammar)


	def execute(self, userdata):
		#change the grammar

		rospy.wait_for_service(self.change_grammar_srv_name, 5)
		rospy.wait_for_service(self.clear_last_command_proxy_srv_name, 3)
		try:
			self.change_grammar("acknowledge_top_level.xml")
		except rospy.ServiceException, e:
			print "Service called failed: %s"%e
				
		SAY('Did you say: ' + userdata.understood_command + ', is this correct?')
		print userdata.understood_command
	
		self.clear_last_command_proxy()
		# wait for the command
		#cs.common_states.set_light_color(cs.common_states.COLOR_GREEN)
		

		rospy.wait_for_service(self.get_last_recognized_speech_srv_name, 3)
		while(True):
			res = self.get_last_recognized_speech_srv()
			if res.keyword == 'yes' or res.keyword == 'no':
				break
			else:
				rospy.sleep(0.2)

		print "Last recognized commmand (ACK) ", res.keyword
		if res.keyword == 'yes':
			cs.common_states.set_light_color(cs.common_states.COLOR_RED)
			return 'yes'
		elif res.keyword == 'no':
			cs.common_states.set_light_color(cs.common_states.COLOR_RED)
			return 'no'	


class say_state(smach.State):
	phrase_to_say = ""

	def __init__(self, phrase_to_say):
		smach.State.__init__(self, outcomes=['success'])
		self.phrase_to_say = phrase_to_say

	def execute(self, userdata):
		SAY(self.phrase_to_say)
		return 'success'

class say_state_dynamic(smach.State):
	prefix = ""
	suffix = ""
	def __init__(self, prefix, suffix):
		smach.State.__init__(self, outcomes=['success'], input_keys=['phrase_in'])	
		self.prefix = prefix
		self.suffix = suffix

	def execute(self, userdata):
		phrase = self.prefix + " " + userdata.phrase_in + " " + self.suffix
		SAY(phrase)
		return 'success'

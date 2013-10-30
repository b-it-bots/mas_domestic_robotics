#!/usr/bin/python

######################### IMPORTS #########################

import roslib
roslib.load_manifest('mdr_common_states')
import rospy
import smach
import smach_ros

from simple_script_server import *
sss = simple_script_server()

import std_msgs.msg
import std_srvs.srv
import actionlib_msgs.msg

import mcr_speech_msgs.srv

light_pub = rospy.Publisher('light_controller/command', ColorRGBA)

red_light = ColorRGBA()
red_light.r = 1.0
red_light.g = 0.0
red_light.b = 0.0
red_light.a = 0.1


green_light = ColorRGBA()
green_light.r = 0.0
green_light.g = 1.0
green_light.b = 0.0
green_light.a = 0.1



######################### GLOBAL SAY TO AVOID SCRIPT SERVER #########################

def SAY(text):
	say_service_proxy = rospy.ServiceProxy('/say', mcr_speech_msgs.srv.say)
	rospy.wait_for_service('/say',30)
	say_service_proxy(text)
	rospy.sleep(0.2)
	clear_last_command_proxy = rospy.ServiceProxy("/mcr_speech/speech_recognition/clear_last_recognized_speech", std_srvs.srv.Empty)
	rospy.wait_for_service("/mcr_speech/speech_recognition/clear_last_recognized_speech",3)
	clear_last_command_proxy()
	


class init_speech(smach.State):
	grammar = 'follow_me.xml'
	def __init__(self, grammarfile):
		smach.State.__init__(self, outcomes=['success', 'failed'])
		self.grammar = grammarfile

	def execute(self, userdata):
		speech_recognition_change_grammar = rospy.ServiceProxy('/mcr_speech/speech_recognition/change_grammar', mcr_speech_msgs.srv.ChangeGrammar)
		rospy.wait_for_service('/mcr_speech/speech_recognition/change_grammar', 5)
		try:
			speech_recognition_change_grammar(self.grammar)
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
		self.get_last_recognized_speech = rospy.ServiceProxy('/mcr_speech/speech_recognition/get_last_recognized_speech', mcr_speech_msgs.srv.GetLastRecognizedSpeech)

	def execute(self, userdata):
		# wait for the command
		light_pub.publish(green_light)
		rospy.wait_for_service('/mcr_speech/speech_recognition/get_last_recognized_speech', 3)
		
		while(True):
			res = self.get_last_recognized_speech()
			self.last_command = res.keyword
			if self.last_command in userdata.commands_to_wait_for:
				break
			else:
				rospy.sleep(0.2)
				
		print self.last_command

		userdata.understood_command = self.last_command 
		light_pub.publish(red_light)	
		return 'success'



class acknowledge_command(smach.State):
	lastCommand = ""

	def __init__(self):
		smach.State.__init__(self, outcomes=['yes','no'], input_keys=['understood_command'])
		self.getLastRecognizedSpeech = rospy.ServiceProxy('/mcr_speech/speech_recognition/get_last_recognized_speech', mcr_speech_msgs.srv.GetLastRecognizedSpeech)
		self.clear_last_command_proxy = rospy.ServiceProxy("/mcr_speech/speech_recognition/clear_last_recognized_speech", std_srvs.srv.Empty)

	def execute(self, userdata):
		rospy.wait_for_service('/mcr_speech/speech_recognition/get_last_recognized_speech', 3)
		rospy.sleep(0.2)
	
		rospy.wait_for_service("/mcr_speech/speech_recognition/clear_last_recognized_speech",3)
		

		SAY('Did you say: ' + userdata.understood_command + ', is this correct?')
		print userdata.understood_command
		rospy.sleep(0.2)
		self.clear_last_command_proxy()
		# wait for the command
		light_pub.publish(green_light)

		while(True):
			res = self.getLastRecognizedSpeech()
			if res.keyword == 'yes' or res.keyword == 'no':
				break
			else:
				rospy.sleep(0.2)

		print "Last recognized commmand (ACK) ", res.keyword
		if res.keyword == 'yes':
			light_pub.publish(red_light)
			return 'yes'
		elif res.keyword == 'no':
			light_pub.publish(red_light)
			return 'no'	


#loads only the Ack grammar
class acknowledge_command_with_loading_grammar(smach.State):
	lastCommand = ""

	def __init__(self):
		smach.State.__init__(self, outcomes=['yes','no'], input_keys=['understood_command'])
		self.getLastRecognizedSpeech = rospy.ServiceProxy('/mcr_speech/speech_recognition/get_last_recognized_speech', mcr_speech_msgs.srv.GetLastRecognizedSpeech)
		self.clear_last_command_proxy = rospy.ServiceProxy("/mcr_speech/speech_recognition/clear_last_recognized_speech", std_srvs.srv.Empty)

	def execute(self, userdata):
		#change the grammar
		speech_recognition_change_grammar = rospy.ServiceProxy('/mcr_speech/speech_recognition/change_grammar', mcr_speech_msgs.srv.ChangeGrammar)
		rospy.wait_for_service('/mcr_speech/speech_recognition/change_grammar', 5)
		rospy.wait_for_service("/mcr_speech/speech_recognition/clear_last_recognized_speech",3)
		try:
			speech_recognition_change_grammar("acknowledge_top_level.xml")
		except rospy.ServiceException, e:
			print "Service called failed: %s"%e
				
		SAY('Did you say: ' + userdata.understood_command + ', is this correct?')
		print userdata.understood_command
	
		self.clear_last_command_proxy()
		# wait for the command
		light_pub.publish(green_light)
		

		rospy.wait_for_service('/mcr_speech/speech_recognition/get_last_recognized_speech', 3)
		while(True):
			res = self.getLastRecognizedSpeech()
			if res.keyword == 'yes' or res.keyword == 'no':
				break
			else:
				rospy.sleep(0.2)

		print "Last recognized commmand (ACK) ", res.keyword
		if res.keyword == 'yes':
			light_pub.publish(red_light)
			return 'yes'
		elif res.keyword == 'no':
			light_pub.publish(red_light)
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

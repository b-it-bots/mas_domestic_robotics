#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2017.09.18

@author: Patrick Nagel
"""

from random import randint

import rospy
import smach
import speech_recognition as sr

from std_msgs.msg import String
from smach_ros import ActionServerWrapper, IntrospectionServer
from mdr_listen_action.msg import ListenAction, ListenResult, ListenFeedback

class InitializeListen(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'processing'],
                             input_keys=['error_message', 'listen_feedback'],
                             output_keys=['listen_feedback', 'init_error_message'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializeListen")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback = ListenFeedback()
            userdata.listen_feedback.status_initialization = "in_progess"
            userdata.listen_feedback.status_wait_for_user_input = "pending"
            userdata.listen_feedback.status_process_input = "pending"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        initialization_successful = True
        if initialization_successful:
            rospy.loginfo("Initialization successful!")
            return 'succeeded'
        else:
            # Example for a possible error message.
            userdata.init_error_message = "Microphones could not be initialized."
            rospy.logerr("Microphones could not be initialized.")
            return 'failed'


class WaitForUserInput(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['input_received', 'input_not_understood', 'no_input_received', 'processing'],
                             input_keys=['listen_feedback', 'accoustic_input'],
                             output_keys=['listen_feedback', 'accoustic_input',
                                          'input_error_message'])
        self.feedback_given = False
        self.input_received = False

    def callback(self, data, userdata):
        rospy.loginfo("I heard: " + data)

        # TODO: SpeechVerifier, here!
        #sv = SpeechVerifier("../../../common/config/dict", "../../../common/config/sentence_pool.txt", data.sentences)
        #userdata.accoustic_input = sv.find_best_match()
        self.input_received = True
        userdata.accoustic_input = data

    def execute(self, userdata):
        rospy.loginfo("Executing state WaitForUserInput")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "in_progress"
            userdata.listen_feedback.status_process_input = "pending"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        recognizer = sr.Recognizer()
        microphone = sr.Microphone()

        with microphone as source:
            recognizer.adjust_for_ambient_noise(source)

        with microphone as source:
            rospy.loginfo("Listening...")
            audio = recognizer.listen(source)
        rospy.loginfo("Got a sound; recognizing...")
        try:
            recognized_speech = recognizer.recognize_google(audio)
            self.callback(recognized_speech, userdata)
        except sr.UnknownValueError:
            userdata.input_error_message = "Input not understood."
            rospy.logerr("Input not understood.")
            return 'input_not_understood'
        except sr.RequestError as e:
            userdata.input_error_message = "No input received."
            rospy.logerr("No input received")
            return 'no_input_received'

        if self.input_received:
            return 'input_received'
            rospy.sleep(rospy.Duration(1))
            timer += 1

        userdata.input_error_message = "No input received."
        rospy.logerr("No input received.")
        return 'no_input_received'


class InitializationError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],
                             input_keys=['error_message_in', 'listen_feedback'],
                             output_keys=['listen_feedback', 'listen_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializationError")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "aborted"
            userdata.listen_feedback.status_wait_for_user_input = "pending"
            userdata.listen_feedback.status_process_input = "pending"
            userdata.listen_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = ListenResult()
        result.success = False
        result.message = "None"
        result.message_type = "None"
        userdata.listen_result = result
        return 'error_detected'


class ProcessInput(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'input_not_understood', 'processing'],
                             input_keys=['accoustic_input', 'listen_feedback',
                                         'input_error_message'],
                             output_keys=['listen_feedback', 'listen_result',
                                          'input_error_message'])
        self.feedback_given = False
        self.feedback_updated = False

    def execute(self, userdata):
        rospy.loginfo("Executing state ProcessInput")
        rospy.loginfo("This is what I think you meant: %s", userdata.accoustic_input)

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "completed"
            userdata.listen_feedback.status_process_input = "in_progress"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        result = ListenResult()
        result.success = True
        result.message = userdata.accoustic_input
        result.message_type = "String"
        userdata.listen_result = result
        return 'succeeded'


class InputError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],
                             input_keys=['error_message_in', 'listen_feedback'],
                             output_keys=['listen_feedback', 'listen_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InputError")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "completed"
            userdata.listen_feedback.status_process_input = "aborted"
            userdata.listen_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = ListenResult()
        result.success = False
        result.message = "None"
        result.message_type = "None"
        userdata.listen_result = result
        return 'error_detected'

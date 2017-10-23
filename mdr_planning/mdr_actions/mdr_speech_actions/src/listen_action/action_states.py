#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2017.09.18

@author: Patrick Nagel
"""

from random import randint

import rospy
import smach

from std_msgs.msg import String
from smach_ros import ActionServerWrapper, IntrospectionServer
from mdr_speech_actions.msg import ListenAction, ListenResult, ListenFeedback

class InitializeListen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],\
        input_keys=['error_message', 'listen_feedback'],\
        output_keys=['listen_feedback', 'init_error_message'])

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializeListen")

        initialization_successful = True

        # give some feedback
        userdata.listen_feedback = ListenFeedback()
        userdata.listen_feedback.status_initialization = "in_progess"
        userdata.listen_feedback.status_wait_for_user_input = "pending"
        userdata.listen_feedback.status_process_input = "pending"
        userdata.listen_feedback.error_detected = False

        if initialization_successful:
            rospy.loginfo("Initialization successful!")
            return 'succeeded'
        else:
            # example for a possible error message
            userdata.init_error_message = "Microphones could not be initialized."
            rospy.logerr("Microphones could not be initialized.")
            return 'failed'

class WaitForUserInput(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['input_received', 'no_input_received', 'processing'],\
        input_keys=['listen_goal', 'listen_feedback', 'accoustic_input'],\
        output_keys=['listen_feedback', 'accoustic_input', 'input_error_message'])
        self.feedback_given = False
        self.input_received = False

    def callback(self, data, userdata):
        rospy.loginfo(rospy.get_caller_id() + " I heard %s ", data.data)
        self.input_received = True
        userdata.accoustic_input = data.data

    def execute(self, userdata):
        rospy.loginfo("Executing state WaitForUserInput")

        # give some feedback
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "in_progress"
            userdata.listen_feedback.status_process_input = "pending"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        timer = 0
        duration = userdata.listen_goal.listen_duration
        while not timer == duration:
            # in order to provide some input: publish to /wait_for_user_input
            rospy.Subscriber("wait_for_user_input", String, self.callback, userdata)
            if self.input_received:
                return 'input_received'
            rospy.sleep(rospy.Duration(1))
            timer += 1

        userdata.input_error_message = "No input received."
        rospy.logerr("No input received.")
        return 'no_input_received'

class InitializationError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],\
        input_keys=['error_message_in', 'listen_feedback'],\
        output_keys=['listen_feedback', 'listen_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializationError")

        # give some feedback
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
        smach.State.__init__(self, outcomes=['succeeded', 'input_not_understood', 'processing'],\
        input_keys=['accoustic_input', 'listen_feedback', 'input_error_message'],\
        output_keys=['listen_feedback', 'listen_result', 'input_error_message'])
        self.feedback_given = False
        self.feedback_updated = False

    def execute(self, userdata):
        rospy.loginfo("Executing state ProcessInput")
        rospy.loginfo("This was the input %s: ", userdata.accoustic_input)

        # give some feedback
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "completed"
            userdata.listen_feedback.status_process_input = "in_progress"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        # simple check
        # - if the sentence starts with a question mark, then it is a question
        if "?" in userdata.accoustic_input:
            # update feedback
            while not self.feedback_updated:
                userdata.listen_feedback.status_initialization = "completed"
                userdata.listen_feedback.status_wait_for_user_input = "completed"
                userdata.listen_feedback.status_process_input = "completed"
                userdata.listen_feedback.error_detected = False
                self.feedback_updated = True
                return 'processing'

            result = ListenResult()
            result.success = True
            result.message = userdata.accoustic_input
            result.message_type = 'question'
            userdata.listen_result = result
            rospy.loginfo("This is the message type %s: ", result.message_type)
            return 'succeeded'

        # simple check, simple case
        # - if the sentence starts with a dot, then it is a questionable statement
        elif "." in userdata.accoustic_input:
            # update feedback
            while not self.feedback_updated:
                userdata.listen_feedback.status_initialization = "completed"
                userdata.listen_feedback.status_wait_for_user_input = "completed"
                userdata.listen_feedback.status_process_input = "completed"
                userdata.listen_feedback.error_detected = False
                self.feedback_updated = True
                return 'processing'

            result = ListenResult()
            result.success = True
            result.message = userdata.accoustic_input
            result.message_type = 'questionable_statement'
            userdata.listen_result = result
            rospy.loginfo("This is the message type %s: ", result.message_type)
            return 'succeeded'

        else:
            userdata.input_error_message = "The accoustic signal could not be identified."
            rospy.logerr("%s", userdata.input_error_message)
            return 'input_not_understood'

class InputError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],\
        input_keys=['error_message_in', 'listen_feedback'],\
        output_keys=['listen_feedback', 'listen_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InputError")

        # give some feedback
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

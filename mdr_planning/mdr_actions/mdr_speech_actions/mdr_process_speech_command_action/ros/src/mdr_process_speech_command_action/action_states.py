#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2018.10.19

@author: Patrick Nagel, Roberto Cai
"""

import rospy
import smach

from mdr_process_speech_command_action.msg import ProcessSpeechFeedback, ProcessSpeechResult

class InitializeProcessSpeechCommand(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'processing'],
                             input_keys=['process_speech_command_feedback'],
                             output_keys=['process_speech_command_feedback', 'init_error_message'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializeProcessSpeechCommand")

        # give some feedback
        while not self.feedback_given:
            userdata.process_speech_command_feedback = ProcessSpeechFeedback()
            userdata.process_speech_command_feedback.status_initialization = "in_progress"
            userdata.process_speech_command_feedback.status_match_command = "pending"
            userdata.process_speech_command_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        initialization_successful = True
        if initialization_successful:
            rospy.loginfo("Initialization successful!")
            return 'succeeded'
        else:
            rospy.logwarn("Initialization failed!")
            return 'failed'


class InitializationError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],
                             input_keys=['process_speech_command_feedback', 'error_message'],
                             output_keys=['process_speech_command_feedback', 'process_speech_command_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializationError")
        # give some feedback
        while not self.feedback_given:
            userdata.process_speech_command_feedback.status_initialization = "aborted"
            userdata.process_speech_command_feedback.status_match_command = "pending"
            userdata.process_speech_command_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = ProcessSpeechResult()
        result.success = False
        userdata.process_speech_command_result = result
        return 'error_detected'


class MatchCommand(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'command_not_found', 'processing'],
                             input_keys=['process_speech_command_feedback', 'process_speech_command_goal', 'process_speech_command_result'],
                             output_keys=['process_speech_command_feedback', 'process_speech_command_result',
                                          'match_error_message'])
        self.feedback_given = False
        self.feedback_updated = False

    def execute(self, userdata):
        rospy.loginfo("Executing state MatchCommand")
        # give some feedback
        while not self.feedback_given:
            userdata.process_speech_command_feedback.status_initialization = "completed"
            userdata.process_speech_command_feedback.status_match_command = "in_progress"
            userdata.process_speech_command_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        starting_command = "store groceries"
        if userdata.process_speech_command_goal.command == starting_command:
            #Do something
            result = ProcessSpeechResult()
            result.success = True
            userdata.process_speech_command_result = result
            rospy.loginfo("%s", userdata.process_speech_command_result)
            return 'succeeded'
        else:
            return "command_not_found"


class MatchError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],
                             input_keys=['process_speech_command_feedback', 'error_message'],
                             output_keys=['process_speech_command_feedback', 'process_speech_command_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state MatchError")
        # give some feedback
        while not self.feedback_given:
            userdata.process_speech_command_feedback.status_initialization = "completed"
            userdata.process_speech_command_feedback.status_match_command = "aborted"
            userdata.process_speech_command_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = ProcessSpeechResult()
        result.success = False
        userdata.process_speech_command_result = result
        return 'error_detected'

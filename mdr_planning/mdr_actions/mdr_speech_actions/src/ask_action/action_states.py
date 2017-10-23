#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2017.09.25

@author: Patrick Nagel
"""

from random import randint

import os
import rospy
import smach

from mdr_speech_actions.msg import AskResult, AskFeedback

class InitializeAsk(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],\
        input_keys=['ask_feedback'],\
        output_keys=['ask_feedback'])

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializeAsk")
        
        initialization_successful = True

        # give some feedback
        userdata.ask_feedback = AskFeedback()
        userdata.ask_feedback.status_initialization = "in_progress"
        userdata.ask_feedback.status_match_question = "pending"
        userdata.ask_feedback.error_detected = False

        if initialization_successful:
            rospy.loginfo("Initialization successful!")
            return 'succeeded'
        else:
            return 'failed'

class InitializationError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],\
        input_keys=['ask_feedback'],\
        output_keys=['ask_feedback', 'ask_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializationError")

        # give some feedback
        while not self.feedback_given:
            userdata.ask_feedback.status_initialization = "aborted"
            userdata.ask_feedback.status_match_question = "pending"
            userdata.ask_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = AskResult()
        result.success = False
        userdata.ask_result = result
        return 'error_detected'

class MatchQuestion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'question_not_found', 'processing'],\
        input_keys=['ask_feedback', 'ask_goal', 'ask_result'],\
        output_keys=['ask_feedback', 'ask_result',\
        'match_error_message'])
        self.feedback_given = False
        self.feedback_updated = False

    def execute(self, userdata):
        rospy.loginfo("Executing state MatchQuestion")
        # give some feedback
        while not self.feedback_given:
            userdata.ask_feedback = AskFeedback()
            userdata.ask_feedback.status_initialization = "completed"
            userdata.ask_feedback.status_match_question = "in_progress"
            userdata.ask_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        matching_line = None
        file_dir = os.path.join(os.path.dirname(__file__), "ask.txt")
        ask_file = open(file_dir, "r")
        for line in ask_file:
            if userdata.ask_goal.triggering_statement in line:
                matching_line = line

        if matching_line is None:
            ask_file.close()
            userdata.match_error_message = "There is no question\
            to this statement."
            rospy.logerr("There is no question to this statement.")
            return 'question_not_found'
        else:
            # update feedback
            while not self.feedback_updated:
                userdata.ask_feedback.status_initialization = "completed"
                userdata.ask_feedback.status_match_question = "completed"
                userdata.ask_feedback.error_detected = False
                self.feedback_updated = True
                return 'processing'
                
            result = AskResult()
            result.success = True
            result.ask_message = matching_line.partition(':')[2]
            ask_file.close()
            userdata.ask_result = result
            rospy.loginfo("%s", userdata.ask_result)
            return 'succeeded'

class MatchError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],\
        input_keys=['ask_feedback', 'error_message'],\
        output_keys=['ask_feedback', 'ask_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state MatchError")

        # give some feedback
        while not self.feedback_given:
            userdata.ask_feedback = AskFeedback()
            userdata.ask_feedback.status_initialization = "completed"
            userdata.ask_feedback.status_match_question = "aborted"
            userdata.ask_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = AskResult()
        result.success = False
        result.ask_message = "Ich habe keine Frage zu der Aussage."
        userdata.ask_result = result
        return 'error_detected'

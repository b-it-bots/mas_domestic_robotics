#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2017.09.23

@author: Patrick Nagel
"""

from random import randint

import os
import rospy
import smach
import rospkg

from mdr_answer_action.msg import AnswerFeedback, AnswerResult

class InitializeAnswer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],\
        input_keys=['answer_feedback'],\
        output_keys=['answer_feedback', 'init_error_message'])

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializeAnswer")
        
        initialization_successful = True

        # give some feedback
        userdata.answer_feedback = AnswerFeedback()
        userdata.answer_feedback.status_initialization = "in_progress"
        userdata.answer_feedback.status_match_answer = "pending"
        userdata.answer_feedback.error_detected = False

        if initialization_successful:
            rospy.loginfo("Initialization successful!")
            return 'succeeded'
        else:
            rospy.logwarn("Initialization failed!")
            return 'failed'

class InitializationError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],\
        input_keys=['answer_feedback', 'error_message'],\
        output_keys=['answer_feedback', 'answer_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializationError")
        # give some feedback
        while not self.feedback_given:
            userdata.answer_feedback.status_initialization = "aborted"
            userdata.answer_feedback.status_match_answer = "pending"
            userdata.answer_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = AnswerResult()
        result.success = False
        userdata.answer_result = result
        return 'error_detected'

class MatchAnswer(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'answer_not_found', 'processing'],\
        input_keys=['answer_feedback', 'answer_goal', 'answer_result'],\
        output_keys=['answer_feedback', 'answer_result',\
        'match_error_message'])
        self.feedback_given = False
        self.feedback_updated = False

    def execute(self, userdata):
        rospy.loginfo("Executing state MatchAnswer")
        # give some feedback
        while not self.feedback_given:
            userdata.answer_feedback.status_initialization = "completed"
            userdata.answer_feedback.status_match_answer = "in_progress"
            userdata.answer_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        # check if an answer to the question exists
        rospack = rospkg.RosPack()
        matching_line = None
        file_dir = os.path.join(rospack.get_path("mdr_answer_action"), "answers.txt")
        answer_file = open(file_dir, "r")
        for line in answer_file:
            if userdata.answer_goal.question in line:
                matching_line = line

        if matching_line is None:
            answer_file.close()
            userdata.match_error_message = "There is no answer\
            to this question."
            rospy.logerr("There is no answer to this question.")
            return 'answer_not_found'
        else:
            # update feedback
            while not self.feedback_updated:
                userdata.answer_feedback.status_initialization = "completed"
                userdata.answer_feedback.status_match_answer = "completed"
                userdata.answer_feedback.error_detected = False
                self.feedback_updated = True
                return 'processing'

            result = AnswerResult()
            result.success = True
            result.answer_message = matching_line.partition(':')[2]
            answer_file.close()
            userdata.answer_result = result
            rospy.loginfo("%s", userdata.answer_result)
            return 'succeeded'

class MatchError(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],\
        input_keys=['answer_feedback', 'error_message'],\
        output_keys=['answer_feedback', 'answer_result'])
        self.feedback_given = False
        
    def execute(self, userdata):
        rospy.loginfo("Executing state MatchError")
        # give some feedback
        while not self.feedback_given:
            userdata.answer_feedback.status_initialization = "completed"
            userdata.answer_feedback.status_match_answer = "aborted"
            userdata.answer_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = AnswerResult()
        result.success = False
        result.answer_message = "Ich habe keine Antwort auf die Frage."
        userdata.answer_result = result
        return 'error_detected'

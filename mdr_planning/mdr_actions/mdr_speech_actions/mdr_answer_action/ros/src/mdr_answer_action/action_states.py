#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2018.10.16

@author: Patrick Nagel
"""

import os
import rospy
import smach
import rospkg

from mdr_answer_action.msg import AnswerFeedback, AnswerResult

class InitializeAnswer(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'processing'],
                             input_keys=['answer_feedback'],
                             output_keys=['answer_feedback', 'init_error_message'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializeAnswer")

        # give some feedback
        while not self.feedback_given:
            userdata.answer_feedback = AnswerFeedback()
            userdata.answer_feedback.status_initialization = "in_progress"
            userdata.answer_feedback.status_match_answer = "pending"
            userdata.answer_feedback.error_detected = False
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
                             input_keys=['answer_feedback', 'error_message'],
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
        smach.State.__init__(self, outcomes=['succeeded', 'answer_not_found', 'processing'],
                             input_keys=['answer_feedback', 'answer_goal', 'answer_result'],
                             output_keys=['answer_feedback', 'answer_result',
                                          'match_error_message'])
        self.feedback_given = False
        self.feedback_updated = False

    def load_pool(self, filename):
        file = open(filename, "r")
        questions_and_answers = []
        for line in file:
            question_and_answer = line.split(":")
            questions_and_answers.append(question_and_answer)
        return questions_and_answers

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
        answer_dir = os.path.join(rospack.get_path("mdr_question_answering"), "ros/config/answers.txt")
        question_answer_pool = self.load_pool(answer_dir)
        questions = [i[0].strip() for i in question_answer_pool]
        answers = [i[1].strip() for i in question_answer_pool]

        answer = None
        for i, question in enumerate(questions):
            if question == userdata.answer_goal.question:
                answer = answers[i]
                rospy.loginfo("You answer: '" + answer + "'.")
                break

        if answer is None:
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
            result.answer_message = answer
            userdata.answer_result = result
            rospy.loginfo("%s", userdata.answer_result)
            return 'succeeded'


class MatchError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],
                             input_keys=['answer_feedback', 'error_message'],
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
        result.answer_message = "I don't have an answer to your question."
        userdata.answer_result = result
        return 'error_detected'

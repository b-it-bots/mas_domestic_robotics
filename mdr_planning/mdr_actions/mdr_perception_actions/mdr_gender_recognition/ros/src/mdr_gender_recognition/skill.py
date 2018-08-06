#!/usr/bin/env python
import rospy
import smach
from smach import StateMachine as sm

from mdr_gender_recognition.msg import GenderRecognitionAction
from mdr_gender_recognition.action_states import (SetupGenderRecognition,
                                                  RecognizeGenders,
                                                  SetActionLibResult)


class GenderRecognitionSkill(smach.StateMachine):
    def __init__(self, timeout=10):
        sm.__init__(self,
                    outcomes=['OVERALL_SUCCESS',
                              'OVERALL_FAILED', 'PREEMPTED'],
                    input_keys=['gender_recognition_goal'],
                    output_keys=['gender_recognition_feedback',
                                 'gender_recognition_result'])

        gender_model_path = rospy.get_param('~gender_model_path', '')
        image_topic = rospy.get_param('~image_topic', '/cam3d/rgb/image_raw')
        labels = {0: 'woman', 1: 'man'}
        image_size = (64, 64, 1)

        with self:
            sm.add('SETUP_GENDER_RECOGNITION', SetupGenderRecognition(),
                   transitions={'succeeded': 'RECOGNIZE_GENDERS',
                                'failed': 'SETUP_GENDER_RECOGNITION'})

            sm.add('RECOGNIZE_GENDERS',
                   RecognizeGenders(gender_model_path=gender_model_path,
                                    image_topic=image_topic,
                                    labels=labels, image_size=image_size),
                   transitions={'succeeded': 'SET_ACTION_LIB_SUCCESS',
                                'failed': 'SET_ACTION_LIB_FAILED'})

            sm.add('SET_ACTION_LIB_FAILED',
                   SetActionLibResult(False),
                   transitions={'succeeded': 'OVERALL_FAILED'})

            sm.add('SET_ACTION_LIB_SUCCESS',
                   SetActionLibResult(True),
                   transitions={'succeeded': 'OVERALL_SUCCESS'})

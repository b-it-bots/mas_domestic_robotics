#!/usr/bin/env python
import rospy
import smach
from smach import StateMachine as sm
from mdr_detect_person.msg import DetectPersonAction
from mdr_detect_person.action_states import (SetupDetectPerson, DetectPerson,
                                             SetActionLibResult)


class DetectPersonSkill(smach.StateMachine):
    def __init__(self, timeout=10):
        sm.__init__(self,
                    outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILED', 'PREEMPTED'],
                    input_keys=['detect_person_goal'],
                    output_keys=['detect_person_feedback',
                                 'detect_person_result'])

        detection_model_path = rospy.get_param('~config_file', '')
        image_topic = rospy.get_param('~image_topic', '/cam3d/rgb/image_raw')

        with self:
            sm.add('SETUP_DETECT_PERSON', SetupDetectPerson(),
                   transitions={'succeeded': 'DETECT_PERSON',
                                'failed': 'SETUP_DETECT_PERSON'})

            sm.add('DETECT_PERSON',
                   DetectPerson(image_topic=image_topic,
                                detection_model_path=detection_model_path),
                   transitions={'succeeded': 'SET_ACTION_LIB_SUCCESS',
                                'failed': 'SET_ACTION_LIB_FAILED'})

            sm.add('SET_ACTION_LIB_FAILED',
                   SetActionLibResult(False),
                   transitions={'succeeded': 'OVERALL_FAILED'})

            sm.add('SET_ACTION_LIB_SUCCESS',
                   SetActionLibResult(True),
                   transitions={'succeeded': 'OVERALL_SUCCESS'})

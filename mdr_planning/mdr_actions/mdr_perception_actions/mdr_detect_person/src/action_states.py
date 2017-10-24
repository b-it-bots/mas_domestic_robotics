#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import cv2

from mdr_detect_person.msg import DetectPersonFeedback, DetectPersonGoal, DetectPersonResult

from utils.inference import detect_faces
from utils.inference import load_detection_model


class SetupDetectPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['detect_person_goal'],
                             output_keys=['detect_person_feedback', 'detect_person_result'])

    def execute(self, userdata):
        start = userdata.detect_person_goal.start

        feedback = DetectPersonFeedback()
        feedback.current_state = 'DETECT_PERSON'
        feedback.text = '[detect_person] detecting faces '
        userdata.detect_person_feedback = feedback

        return 'succeeded'

class DetectPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['detect_person_goal'], outcomes=['succeeded', 'failed'])
        
    def execute(self, userdata):

        detection_model_path = '../trained_models/detection_models/haarcascade_frontalface_default.xml'

        # hyper-parameters for bounding boxes shape
        frame_window = 10

        # loading models
        face_detection = load_detection_model(detection_model_path)

        # starting video streaming
        cv2.namedWindow('window_frame')
        video_capture = cv2.VideoCapture(0)
        while True:

            bgr_image = video_capture.read()[1]
            gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            faces = detect_faces(face_detection, gray_image)

        feedback =  DetectPersonFeedback()
        feedback.faces = faces
        userdata.detect_person_feedback = feedback

        return 'succeeded'

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['detect_person_goal'],
                             output_keys=['detect_person_feedback', 'detect_person_result'])
        self.result = result

    def execute(self, userdata):
        result = DetectPersonResult()
        result.success = self.result
        userdata.detect_person_result = result
        return 'succeeded'

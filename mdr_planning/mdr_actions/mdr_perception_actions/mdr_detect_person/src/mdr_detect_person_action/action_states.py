#!/usr/bin/python3

import rospy
import smach
import smach_ros
import actionlib
import cv2

from mdr_detect_person.msg import DetectPersonFeedback, DetectPersonGoal, DetectPersonResult
from sensor_msgs.msg import Image

from inference import detect_faces
from inference import load_detection_model


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
        rospy.Subscriber("/cam3d/rgb/image_raw", Image, self.callback)
        
    def execute(self, userdata):
        # model for 
        detection_model_path = '../trained_models/detection_models/haarcascade_frontalface_default.xml'

        # loading models
        face_detection = load_detection_model(detection_model_path)

        for i in xrange(1,1000):
            bgr_image = self.video_capture.read()[1]
            gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            faces = detect_faces(face_detection, gray_image)

        if np.size(faces) != 0:

            feedback =  DetectPersonFeedback()
            feedback.faces = faces
            userdata.detect_person_feedback = feedback
            return 'succeeded'
        else:
            return 'failure'

    def callback(self, data):
        rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.video_capture = data.data

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

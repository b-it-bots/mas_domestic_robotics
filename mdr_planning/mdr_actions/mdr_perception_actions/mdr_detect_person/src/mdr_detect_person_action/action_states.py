#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import cv2
import numpy as np

from mdr_detect_person.msg import DetectPersonFeedback, DetectPersonGoal, DetectPersonResult
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from mdr_detect_person_action.inference import detect_faces
from mdr_detect_person_action.inference import load_detection_model


class SetupDetectPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['detect_person_goal'],
                             output_keys=['detect_person_feedback', 'detect_person_result'])

    def execute(self, userdata):
        start = userdata.detect_person_goal.start
        print (start)
        if start:
            feedback = DetectPersonFeedback()
            feedback.current_state = 'DETECT_PERSON'
            feedback.text = '[detect_person] detecting faces '
            userdata.detect_person_feedback = feedback
            return 'succeeded'
        else:
            return 'failed'
                

class DetectPerson(smach.State):
    def __init__(self):
        smach.State.__init__(self, input_keys=['detect_person_goal'], outcomes=['succeeded', 'failed'])
        self.bridge = CvBridge()
        self.image_subscriber = rospy.Subscriber("/cam3d/rgb/image_raw", Image, self.callback)
                # model for 
        detection_model_path = '/home/gabych/face_classification/trained_models/detection_models/haarcascade_frontalface_default.xml'

        # loading models
        self.face_detection = load_detection_model(detection_model_path)
        

    def execute(self, userdata):

        for i in iter(range(1,1000)): #xrange(1,1000):
            try:
                bgr_image = self.video_capture#.read()[1]
                gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
                #rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                faces = detect_faces(self.face_detection, gray_image)
                number_of_faces = np.size(faces,0)
                print (number_of_faces)
            except:
                number_of_faces = 0
                print ("Ops")

        if number_of_faces != 0:
            feedback =  DetectPersonFeedback()
            feedback.faces = faces
            userdata.detect_person_feedback = feedback
            return 'succeeded'
        else:
            return 'failed'

    def callback(self, data):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s")# data)
        self.video_capture = self.convert_image(data)
        #print (type(self.video_capture), "videocapture")

    def convert_image(self, ros_image):
        # Use cv_bridge() to convert the ROS image to OpenCV format
        #print (type(ros_image))  
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")     

        return np.array(cv_image, dtype=np.uint8)


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

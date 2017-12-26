#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from mdr_detect_person.msg import DetectPersonFeedback, DetectPersonGoal, DetectPersonResult
from mdr_perception_msgs.msg import FaceBoundingBox

from mdr_detect_person.inference import detect_faces
from mdr_detect_person.inference import load_detection_model


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
            feedback.text = '[detect_person] detecting faces'
            userdata.detect_person_feedback = feedback
            return 'succeeded'
        else:
            return 'failed'


class DetectPerson(smach.State):
    def __init__(self, timeout=120., image_topic='/cam3d/rgb/image_raw', detection_model_path=''):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['detect_person_goal', 'number_of_faces',
                                          'bounding_boxes'],
                             output_keys=['detect_person_feedback', 'number_of_faces',
                                          'bounding_boxes'])
        self.timeout = timeout
        self.bridge = CvBridge()
        self.image_publisher =  rospy.Publisher(image_topic, Image, queue_size=1)

        # loading model
        self.face_detection = load_detection_model(detection_model_path)

    def execute(self, userdata):
        userdata.bounding_boxes = []
        self.input_image = self.convert_image(userdata.detect_person_goal.image)
        try:
            bgr_image = self.input_image
            gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            faces = detect_faces(self.face_detection, gray_image)
            userdata.number_of_faces = np.size(faces, 0)
        except:
            userdata.number_of_faces = 0

        if userdata.number_of_faces != 0:
            for face_coordinates in faces:
                bounding_box = FaceBoundingBox()
                print type(face_coordinates)
                bounding_box.bounding_box_coordinates = face_coordinates.tolist()
                userdata.bounding_boxes.append(bounding_box)

                x, y, w, h = face_coordinates
                rgb_cv2 = cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0,0,255), 2)
            output_ros_image = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
            self.image_publisher.publish(output_ros_image)
            return 'succeeded'
        else:
            return 'failed'

    def convert_image(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        return np.array(cv_image, dtype=np.uint8)

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['number_of_faces', 'bounding_boxes'],
                             output_keys=['detect_person_result','detect_person_feedback'])
        self.result = result

    def execute(self, userdata):
        result = DetectPersonResult()
        result.success = self.result
        result.number_of_faces = userdata.number_of_faces
        result.bounding_boxes = userdata.bounding_boxes
        userdata.detect_person_result = result
        return 'succeeded'

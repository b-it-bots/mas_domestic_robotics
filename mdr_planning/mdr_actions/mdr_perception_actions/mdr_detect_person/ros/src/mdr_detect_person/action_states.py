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

from mdr_detect_person.inference import detect_faces
from mdr_detect_person.inference import load_detection_model
from mdr_detect_person.inference import draw_bounding_box


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
        smach.State.__init__(self, input_keys=['detect_person_goal'], outcomes=['succeeded', 'failed'], output_keys=['detect_person_feedback'])
        self.bridge = CvBridge()
        self.image_publisher =  rospy.Publisher("/cam3d/rgb/image_raw", Image,queue_size=1)
        self.image_subscriber = rospy.Subscriber("/cam3d/rgb/image_raw", Image, self.callback)
        # model for detect faces
        detection_model_path = rospy.get_param("~config_file")
        # loading models
        self.face_detection = load_detection_model(detection_model_path)

    def execute(self, userdata):
        centers_of_faces = []
        for i in iter(range(1, 10)):
            try:
                bgr_image = self.video_capture                      
                gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
                rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
                faces = detect_faces(self.face_detection, gray_image)
                number_of_faces = np.size(faces, 0)
            except:
                number_of_faces = 0

        if number_of_faces != 0:
            feedback = DetectPersonFeedback()
            feedback.bounding_boxes = list(faces)
            feedback.number_of_faces = number_of_faces 

            for face_coordinates in faces:
                    x, y, w, h = face_coordinates
                    rgb_cv2 = cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0,0,255), 2)
                    output_ros_image = self.bridge.cv2_to_imgmsg(rgb_cv2, 'bgr8')
                    self.image_publisher.publish(output_ros_image)
                    centers_of_faces.append([(x+w)/2.0,(y+h)/2.0])
            feedback.center_of_face = centers_of_faces
            print (feedback)
            userdata.detect_person_feedback = feedback
            return 'succeeded'
        else:
            return 'failed'

    def callback(self, data):
        self.video_capture = self.convert_image(data)

    def convert_image(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        return np.array(cv_image, dtype=np.uint8)

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['detect_person_result'])
        self.result = result

    def execute(self, userdata):
        result = DetectPersonResult()
        result.success = self.result
        userdata.detect_person_result = result
        return 'succeeded'

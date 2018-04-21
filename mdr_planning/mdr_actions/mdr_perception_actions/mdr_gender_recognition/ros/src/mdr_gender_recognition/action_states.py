#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
from sensor_msgs.msg import Image

import numpy as np
import cv2
import tensorflow as tf
from keras.models import load_model
from cv_bridge import CvBridge, CvBridgeError

from mdr_gender_recognition.msg import GenderRecognitionFeedback, GenderRecognitionResult


class SetupGenderRecognition(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['gender_recognition_goal'],
                             output_keys=['gender_recognition_feedback', 'gender_recognition_result'])

    def execute(self, userdata):
        feedback = GenderRecognitionFeedback()
        feedback.current_state = 'RECOGNIZE_GENDERS'
        feedback.message = '[gender_recognition] Recognizing gender'
        userdata.gender_recognition_feedback = feedback
        return 'succeeded'


class RecognizeGenders(smach.State):
    def __init__(self, timeout=120.0, image_topic='/cam3d/rgb/image_raw',
                 gender_model_path=None, labels=dict(), image_size=(0, 0, 0)):
        smach.State.__init__(self, input_keys=['gender_recognition_goal', 'genders'],
                             output_keys=['gender_recognition_feedback', 'genders'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.labels = labels
        self.image_size = image_size
        self.nop = np.array([None])
        self.image_publisher = rospy.Publisher(image_topic, Image, queue_size=1)
        self.bridge = CvBridge()

        try:
            # loading model
            self.gender_model = load_model(gender_model_path)
            print('Model ' + gender_model_path + ' loaded successfully')

            # the following two lines are necessary for avoiding https://github.com/keras-team/keras/issues/2397
            self.gender_model._make_predict_function()
            self.computation_graph = tf.get_default_graph()
        except:
            print('Failed to load model ', gender_model_path)

    def execute(self, userdata):
        number_of_faces = userdata.gender_recognition_goal.number_of_faces
        bounding_boxes = userdata.gender_recognition_goal.bounding_boxes
        userdata.genders = list()

        rgb_image = self.ros2cv(userdata.gender_recognition_goal.image)
        gray_image = self.rgb2gray(rgb_image)
        for face in bounding_boxes:
            x, y, w, h = face.bounding_box_coordinates
            face = gray_image[y: (y + h), x: (x + w)]  # check if it is not rgb
            face = cv2.resize(face, self.image_size[0:2])
            face = np.expand_dims(face, 0)
            face = np.expand_dims(face, -1)
            face = self.preprocess_image(face)
            recognized_gender = self.recognize_gender(face)
            userdata.genders.append(recognized_gender)
            rgb_cv2 = cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            cv2.putText(rgb_image, recognized_gender, (x, y - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0),
                        1, cv2.LINE_AA)
        output_ros_image = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
        self.image_publisher.publish(output_ros_image)
        return 'succeeded'

    def preprocess_image(self, image):
        image = image / 255.0
        return image

    def recognize_gender(self, face):
        label = -1
        with self.computation_graph.as_default():
            class_predictions = self.gender_model.predict(face)
            label = self.labels[np.argmax(class_predictions)]
        return label

    def ros2cv(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        return np.array(cv_image, dtype=np.uint8)

    def rgb2gray(self, image):
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        return gray_image


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['gender_recognition_goal', 'genders'],
                             output_keys=['gender_recognition_feedback', 'gender_recognition_result'])
        self.result = result

    def execute(self, userdata):
        result = GenderRecognitionResult()
        result.success = self.result
        result.genders = userdata.genders
        userdata.gender_recognition_result = result
        return 'succeeded'

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

from mdr_recognize_emotion_action.msg import (RecognizeEmotionFeedback,
                                              RecognizeEmotionResult)


class SetupRecognizeEmotion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['recognize_emotion_goal'],
                             output_keys=['recognize_emotion_feedback',
                                          'recognize_emotion_result'])

    def execute(self, userdata):
        feedback = RecognizeEmotionFeedback()
        feedback.current_state = 'RECOGNIZE_EMOTION'
        feedback.message = '[recognize_emotion] Recongnizing emotion'
        userdata.recognize_emotion_feedback = feedback
        return 'succeeded'


class RecognizeEmotion(smach.State):
    def __init__(self, timeout=120.0, image_topic='/cam3d/rgb/image_raw',
                 emotion_model_path=None, labels=dict(), image_size=(0, 0, 0)):
        smach.State.__init__(self, input_keys=['recognize_emotion_goal',
                                               'emotions'],
                             output_keys=['recognize_emotion_feedback',
                                          'emotions'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.labels = labels
        self.image_size = image_size
        self.image_publisher = rospy.Publisher(image_topic, Image, queue_size=1)
        self.bridge = CvBridge()

        try:
            self.emotion_model = load_model(emotion_model_path)
            print('Model ' + emotion_model_path + ' loaded successfully')

            # the following two lines (and the with... line in predict_emotion)
            # are necessary for avoiding
            # https://github.com/keras-team/keras/issues/2397
            self.emotion_model._make_predict_function()
            self.computation_graph = tf.get_default_graph()
        except:
            print('Failed to load model ', emotion_model_path)

    def execute(self, userdata):
        number_of_faces = userdata.recognize_emotion_goal.number_of_faces
        bounding_boxes = userdata.recognize_emotion_goal.bounding_boxes
        userdata.emotions = list()

        rgb_image = self.ros2cv(userdata.recognize_emotion_goal.image)
        gray_image = self.rgb2gray(rgb_image)
        for face in bounding_boxes:
            x, y, w, h = face.bounding_box_coordinates
            face = gray_image[y: (y + h), x: (x + w)]
            face = cv2.resize(face, self.image_size[0:2])
            face = np.expand_dims(face, 0)
            face = np.expand_dims(face, -1)
            face = self.preprocess_image(face)
            predicted_emotion = self.predict_emotion(face)
            userdata.emotions.append(predicted_emotion)

            rgb_cv2 = cv2.rectangle(rgb_image, (x, y), (x + w, y + h),
                                    (0, 0, 255), 2)
            cv2.putText(rgb_image, predicted_emotion, (x, y - 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0),
                        1, cv2.CV_AA)
        output_ros_image = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
        self.image_publisher.publish(output_ros_image)
        return 'succeeded'

    def preprocess_image(self, image):
        image = image / 255.0
        return image

    def predict_emotion(self, face):
        label = -1
        with self.computation_graph.as_default():
            class_predictions = self.emotion_model.predict(face)
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
                             input_keys=['recognize_emotion_goal', 'emotions'],
                             output_keys=['recognize_emotion_feedback',
                                          'recognize_emotion_result'])
        self.result = result

    def execute(self, userdata):
        result = RecognizeEmotionResult()
        result.success = self.result
        result.emotions = userdata.emotions
        userdata.recognize_emotion_result = result
        return 'succeeded'

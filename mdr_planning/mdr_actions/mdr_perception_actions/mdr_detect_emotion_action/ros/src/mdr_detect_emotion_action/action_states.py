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

from mdr_detect_emotion_action.msg import DetectEmotionFeedback, DetectEmotionResult
from mdr_detect_person.msg import DetectPersonAction, DetectPersonGoal

class SetupDetectEmotion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['detect_emotion_goal'],
                             output_keys=['detect_emotion_feedback', 'detect_emotion_result'])

    def execute(self, userdata):
        feedback = DetectEmotionFeedback()
        feedback.current_state = 'DETECT_EMOTION'
        feedback.message = '[detect_emotion] Detecting emotion'
        userdata.detect_emotion_feedback = feedback
        return 'succeeded'


class DetectEmotion(smach.State):
    def __init__(self, timeout=120.0, image_topic='/cam3d/rgb/image_raw',
                 emotion_model_path=None, labels=dict(), image_size=(0, 0, 0),
                 detect_person_server='/mdr_actions/detect_person_server'):
        smach.State.__init__(self, input_keys=['detect_emotion_goal', 'number_of_people',
                                               'bounding_boxes', 'emotions'],
                             output_keys=['detect_emotion_feedback', 'number_of_people',
                                          'bounding_boxes', 'emotions'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.labels = labels
        self.image_size = image_size
        self.image_publisher = rospy.Publisher(image_topic, Image, queue_size=1)
        self.bridge = CvBridge()

        self.detect_person_client = actionlib.SimpleActionClient(detect_person_server,
                                                                 DetectPersonAction)
        self.detect_person_client.wait_for_server()

        try:
            self.emotion_model = load_model(emotion_model_path)
            print('Model ' + emotion_model_path + ' loaded successfully')

            # the following two lines (and the with... line in predict_emotion)
            # are necessary for avoiding https://github.com/keras-team/keras/issues/2397
            self.emotion_model._make_predict_function()
            self.computation_graph = tf.get_default_graph()
        except:
            print('Failed to load model ', emotion_model_path)

    def execute(self, userdata):
        goal = DetectPersonGoal()
        goal.start = True
        goal.image = userdata.detect_emotion_goal.image

        self.detect_person_client.send_goal(goal)
        self.detect_person_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        detect_person_result = self.detect_person_client.get_result()

        userdata.number_of_people = detect_person_result.number_of_faces
        userdata.bounding_boxes = detect_person_result.bounding_boxes
        userdata.emotions = list()
        if detect_person_result and detect_person_result.success:
            rgb_image = self.ros2cv(userdata.detect_emotion_goal.image)
            gray_image = self.rgb2gray(rgb_image)

            for face in detect_person_result.bounding_boxes:
                x, y, w, h = face.bounding_box_coordinates
                face = gray_image[y : (y + h), x : (x + w)]
                face = cv2.resize(face, self.image_size[0:2])
                face = np.expand_dims(face, 0)
                face = np.expand_dims(face, -1)
                face = self.preprocess_image(face)
                predicted_emotion = self.predict_emotion(face)
                userdata.emotions.append(predicted_emotion)

                rgb_cv2 = cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0,0,255), 2)
                cv2.putText(rgb_image, predicted_emotion, (x, y - 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0),
                            1, cv2.CV_AA)
            output_ros_image = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
            self.image_publisher.publish(output_ros_image)
            return 'succeeded'
        else:
            return 'failed'

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
                             input_keys=['detect_emotion_goal', 'number_of_people',
                                         'bounding_boxes', 'emotions'],
                             output_keys=['detect_emotion_feedback', 'detect_emotion_result'])
        self.result = result

    def execute(self, userdata):
        result = DetectEmotionResult()
        result.success = self.result
        result.bounding_boxes = userdata.bounding_boxes
        result.number_of_people = userdata.number_of_people
        result.emotions = userdata.emotions
        userdata.detect_emotion_result = result
        return 'succeeded'

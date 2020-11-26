from importlib import import_module
import numpy as np
import cv2

import rospy
import actionlib

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_recognize_gesture.pointing_gesture_recognizer import PointingGestureRecognizer
from mdr_recognize_gesture_action.msg import RecognizeGestureGoal, RecognizeGestureResult

from mas_perception_libs import ImageDetectionKey, ImageDetectorBase
from mas_perception_libs.utils import cloud_msg_to_image_msg, cloud_msg_to_cv_image

from ssd_keras_ros import SSDKerasObjectDetector


class RecognizeGestureSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gesture_type=None,
                 openpose_models_dir='/home/lucy/.models/openpose_models/',
                 pointcloud_topic='/mdr_perception/rectified_points',
                 ssd_detector_class_file='',
                 ssd_detector_kwargs_file=''):
        super(RecognizeGestureSM, self).__init__('RecognizeGesture', [], max_recovery_attempts)

        self.timeout = timeout
        self.gesture_type = gesture_type
        self.openpose_models_dir = openpose_models_dir
        self.pointcloud_topic = pointcloud_topic
        self.ssd_detector_class_file = ssd_detector_class_file
        self.ssd_detector_kwargs_file = ssd_detector_kwargs_file

        self.gesture_classifier = GestureClassifier(gesture_distance_threshold=50.,
                                                    model_path=self.openpose_models_dir,
                                                    gesture_types=['waving', 'nodding', 'shaking_head', 'go_away', 'come_closer', 'pointing'],
                                                    known_example_data_dir='../data/gesture_examples/')

        self.pointing_gesture_recognizer = PointingGestureRecognizer(60., self.openpose_models_dir, False)

    def init(self):
        self.detector = SSDKerasObjectDetector(class_file=self.ssd_detector_class_file, 
                                               model_kwargs_file=self.ssd_detector_kwargs_file)
        return FTSMTransitions.INITIALISED

    def classify_gesture(self):
        # TODO: implement gesture classification
        pass

    def running(self):
        cloud_msg = rospy.wait_for_message(self.pointcloud_topic, PointCloud2)

        rospy.loginfo('[recognize_gesture] Recognizing gesture')
        if self.gesture_type is None:
            rospy.loginfo('[recognize_gesture] Classifying gesture...')
            gesture_data = gesture_classifier.capture_gesture_data()
            self.gesture_type = gesture_classifier.classify_gesture(gesture_data)

            if self.gesture_type is None:
                rospy.loginfo('[recognize_gesture] Gesture not recognized!')
                return FTSMTransitions.DONE
            else:
                rospy.loginfo('This is most likely a {} gesture'.format(self.gesture_type))

        if self.gesture_type == 'pointing':
            predictions = detector.detect([cloud_msg_to_image_msg(cloud_msg)])[0]
            bbs, classes, _ = ImageDetectorBase.prediction_to_bounding_boxes(predictions)
            print('Object classes:', classes)

            image = cloud_msg_to_cv_image(cloud_msg)
            success, obj_index, image = gesture_recognizer.get_object_pointed_to(bbs, image)
            if obj_index is not None:
                print('Pointing to object:', obj_index)
                cv2.imwrite('test_image', image)

        return FTSMTransitions.DONE

    def recovering(self):
        ## TODO: implement any recovery behaviours here
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = HandleOpenResult()
        result.success = success
        return result

from __future__ import print_function

import cv2
import numpy as np
from mdr_recognize_gesture.gesture_classifier import GestureClassifier


float_formatter = "{:.3f}".format
np.set_printoptions(formatter={'float_kind': float_formatter})

if __name__ == '__main__':
    gesture_classifier = GestureClassifier(gesture_distance_threshold=50., 
                                           model_path='/home/afaisal/.models/openpose_models/',
                                           gesture_types=['waving', 'nodding', 'shaking_head', 'go_away', 'come_closer', 'pointing'],
                                           known_example_data_dir='data/gesture_examples/')
    gesture_data = gesture_classifier.capture_gesture_data()
    gesture_classifier.classify_gesture(gesture_data)

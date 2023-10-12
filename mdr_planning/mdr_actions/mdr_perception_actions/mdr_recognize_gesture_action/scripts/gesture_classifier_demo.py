from __future__ import print_function
import argparse

import cv2
import numpy as np

from mdr_recognize_gesture.gesture_classifier import GestureClassifier


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--video_device', '-v', type=str,
                        default='0', help='The video device. For a camera,'
                        'provide its id, for e.g. 0. For a video file, provide'
                        'its path')
    args = parser.parse_args()

    gesture_classifier = GestureClassifier(gesture_distance_threshold=50., 
                                           model_path='/home/afaisal/.models/openpose_models/',
                                           gesture_types=['waving', 'nodding', 'shaking_head', 'go_away', 'come_closer', 'pointing'],
                                           known_example_data_dir='../data/gesture_examples/')
    gesture_data = gesture_classifier.capture_gesture_data(args.video_device)
    most_likely_class = gesture_classifier.classify_gesture(gesture_data)

    if most_likely_class is None:
        print('Gesture not recognized!')
    else:
        print('This is most likely a {} gesture'.format(most_likely_class))

import os
import glob
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError


def get_classes_in_data_dir(data_dir):
    classes = []
    for subdir in sorted(os.listdir(data_dir)):
        if os.path.isdir(os.path.join(data_dir, subdir)):
            classes.append(subdir)

    return classes


def process_image_message(rgb_image, cv_bridge, target_size=None, func_preprocess_img=None):
    np_image = None
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(rgb_image, desired_encoding="passthrough")
        if target_size is not None:
            cv_image = cv2.resize(cv_image, target_size)
        np_image = np.asarray(cv_image)
        np_image = np_image.astype(float)                   # preprocess needs float64 and img is uint8
        if func_preprocess_img is not None:
            np_image = func_preprocess_img(np_image)        # Normalize the data
    except CvBridgeError as e:
        rospy.logerr('error converting to CV image: ' + str(e))
    return np_image


def case_insensitive_glob(pattern):
    def either(c):
        return '[%s%s]' % (c.lower(), c.upper()) if c.isalpha() else c
    return glob.glob(''.join(map(either, pattern)))

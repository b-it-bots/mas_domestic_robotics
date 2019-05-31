#!/usr/bin/env python

import os
import numpy as np

from rospkg import RosPack

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from mas_perception_libs import ImageDetectionKey, ImageDetectorBase
from mas_perception_libs.utils import cloud_msg_to_image_msg, cloud_msg_to_cv_image, \
                                      crop_cloud_to_xyz, draw_labeled_boxes
from ssd_keras_ros import SSDKerasObjectDetector


class FindPeople(object):

    def __init__(self):
        pass


    @staticmethod
    def detect(cloud_msg):
        # Transform point cloud to base link
        #cloud_msg = transform_cloud_with_listener(cloud_msg, '/base_link', tf.TransformListener())

        # use RosPack to point to appropriate configuration files
        rp = RosPack()
        package_path = rp.get_path('ssd_keras_ros')
        class_ann_file = os.path.join(package_path, 'models', 'coco_classes.yml')
        kwargs_file = os.path.join(package_path, 'models', 'ssd_keras_object_detector_kwargs.yml')

        # create SSDKerasObjectDetector object and call detection on
        detector = SSDKerasObjectDetector(class_file=class_ann_file, model_kwargs_file=kwargs_file)
        predictions = detector.detect([cloud_msg_to_image_msg(cloud_msg)])[0]

        # Get bounding boxes
        bb2ds = ImageDetectorBase.prediction_to_bounding_boxes(predictions)[0]

        # Filter results for people
        predictions, bb2ds = FindPeople.filter_people(predictions, bb2ds)

        # Get poses
        poses = FindPeople.get_people_poses(cloud_msg, predictions, bb2ds)

        return predictions, bb2ds, poses


    @staticmethod
    def filter_people(predictions, bounding_boxes):
        people_preds = []
        people_bbs = []

        for i, _ in enumerate(predictions):
            pred = predictions[i]
            bb2d = bounding_boxes[i]

            if pred[ImageDetectionKey.CLASS] == 'person':
                people_preds.append(pred)
                people_bbs.append(bb2d)

        return people_preds, people_bbs


    @staticmethod
    def get_people_poses(cloud_msg, predictions, bounding_boxes):
        poses = []

        for i, _ in enumerate(predictions):
            bb2d = bounding_boxes[i]

            cropped_cloud = crop_cloud_to_xyz(cloud_msg, bb2d)
            mean_coords = np.nanmean(np.reshape(cropped_cloud, (-1, 3)), axis=0)
            mean_ps = PoseStamped()
            mean_ps.header = Header(frame_id=cloud_msg.header.frame_id)
            mean_ps.pose = Pose(Point(mean_coords[0], mean_coords[1], mean_coords[2]), Quaternion())
            poses.append(mean_ps)

        return poses


    @staticmethod
    def render_image_with_detections(cloud_msg, bounding_boxes):
        cv_image = cloud_msg_to_cv_image(cloud_msg)
        image = draw_labeled_boxes(cv_image, bounding_boxes)
        return image

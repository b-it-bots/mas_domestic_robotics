#!/usr/bin/env python

import os

import numpy as np
import pcl
import face_recognition
from sklearn.impute import SimpleImputer

import rospy
from rospkg import RosPack
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from mas_perception_libs import ImageDetectionKey, ImageDetectorBase
from mas_perception_libs.utils import cloud_msg_to_image_msg, cloud_msg_to_cv_image, \
                                      crop_cloud_to_xyz, crop_organized_cloud_msg
from mas_perception_libs.visualization import draw_labeled_boxes
from ssd_keras_ros import SSDKerasObjectDetector


class FindPeople(object):
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

            obj_coords = crop_cloud_to_xyz(cloud_msg, bb2d)
            imputer = SimpleImputer(missing_values=np.nan, strategy='most_frequent')
            imputer.fit(obj_coords.reshape(obj_coords.shape[0]*obj_coords.shape[1], obj_coords.shape[2]))
            obj_coords_without_nans = imputer.transform(obj_coords.reshape(obj_coords.shape[0]*obj_coords.shape[1], obj_coords.shape[2]))

            ## PCL Outlier Removal:
            cloud_without_nans = pcl.PointCloud(obj_coords_without_nans.astype(np.float32))
            voxel_grid_filter = cloud_without_nans.make_voxel_grid_filter()
            voxel_grid_filter.set_leaf_size(0.01, 0.01, 0.01)
            cloud_filtered = voxel_grid_filter.filter()

            passthrough_filter = cloud_filtered.make_passthrough_filter()
            passthrough_filter.set_filter_field_name("z")
            passthrough_filter.set_filter_limits(0., 2.5)
            cloud_filtered = passthrough_filter.filter()
            cloud_filtered_array = cloud_filtered.to_array()

            mean_coords = np.nanmean(cloud_filtered_array, axis=0)
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

    @staticmethod
    def extract_face_image(image_array):
        try:
            top, right, bottom, left = face_recognition.face_locations(image_array)[0]
            rospy.loginfo('[find_people] Successfully extracted face from person image.')
            return image_array[top:bottom, left:right]
        except IndexError:
            rospy.logwarn('[find_people] Failed to extract face from person image!')
            return None

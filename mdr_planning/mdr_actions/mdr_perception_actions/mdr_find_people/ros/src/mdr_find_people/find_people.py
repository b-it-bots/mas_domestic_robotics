#!/usr/bin/env python

import numpy as np
import pcl
import face_recognition
from sklearn.impute import SimpleImputer
import torch

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion

from mas_perception_libs import ImageDetectionKey, ImageDetectorBase, TorchImageDetector
from mas_perception_libs.utils import cloud_msg_to_image_msg, cloud_msg_to_cv_image, \
                                      crop_cloud_to_xyz, crop_organized_cloud_msg
from mas_perception_libs.visualization import draw_labeled_boxes

import dataset_interface.object_detection.transforms as T


class FindPeople(object):
    @staticmethod
    def detect(cloud_msg, detector, detector_device, class_annotations, detection_threshold):
        img = cloud_msg_to_cv_image(cloud_msg)
        transform = T.ToTensor()
        img, _ = transform(img, None)

        rospy.loginfo('Running detector...')
        with torch.no_grad():
            predictions = detector([img.to(detector_device)])

        # Get bounding boxes
        rospy.loginfo('Processing detections...')
        detections = TorchImageDetector.process_predictions(predictions=predictions[0],
                                                            classes=class_annotations,
                                                            detection_threshold=detection_threshold)

        rospy.loginfo('Extracting bounding boxes...')
        bounding_boxes = ImageDetectorBase.prediction_to_bounding_boxes(detections)[0]

        rospy.loginfo('Extracting people detections...')
        person_detections, person_bb2ds = FindPeople.filter_people(detections,
                                                                   bounding_boxes)

        if len(person_detections) == 1:
            rospy.loginfo('Found one person')
        else:
            rospy.loginfo('Found {0} people'.format(len(person_detections)))

        person_poses = []
        if person_detections:
            rospy.loginfo('Extracting person poses...')
            person_poses = FindPeople.get_people_poses(cloud_msg,
                                                       person_detections,
                                                       person_bb2ds)
        rospy.loginfo('Person detection complete')
        return (person_detections, person_bb2ds, person_poses)

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

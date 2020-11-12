#!/usr/bin/env python

from __future__ import print_function
import os
import time
import copy

import cv2
import numpy as np

from fastdtw import fastdtw
from openpose import pyopenpose as op
from scipy.spatial.distance import euclidean
from scipy.ndimage import median_filter, gaussian_filter1d


class GestureClassifier(object):
    def __init__(self, gesture_distance_threshold=None, 
                 model_path='/home/lucy/.models/openpose_models/',
                 gesture_types=['waving', 'nodding', 'shaking_head', 'go_away', 'come_closer', 'pointing'],
		 known_example_data_dir='data/gesture_examples/', 
		 debug=False):
        self.gesture_distance_threshold = gesture_distance_threshold
	self.known_example_data_dir = known_example_data_dir
        self.debug = debug

	self.gesture_example_data_dict = dict(zip(gesture_types, [[] for _ in range(len(gesture_types))]))
        self.gesture_example_pc_indices_dict = copy.deepcopy(self.gesture_example_data_dict)        # pc = principal component i.e. most varying component
	self.gesture_distances_dict = copy.deepcopy(self.gesture_example_data_dict)

        self.video_device = cv2.VideoCapture(0)

        self._load_known_example_data()

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure({"model_folder" : model_path})
        self.opWrapper.start()

    def capture_gesture_data(self, capture_duration=5., save_data=False, filename=None):
        end_time = time.time() + capture_duration
	full_poses = []

        print('Starting gesture recording...')
        while(time.time() < end_time): 
            ret, frame = self.video_device.read()

            datum = op.Datum()
            datum.cvInputData = frame
            self.opWrapper.emplaceAndPop([datum])

            if datum.poseKeypoints.shape == ():
                if self.debug:
                    print('Body probably too close to adequately estimate pose.' 
                            'Please move away from the camera.')

            else:
                if datum.poseKeypoints.shape[0] > 1:
                    datum.poseKeypoints = datum.poseKeypoints[0, :, :][np.newaxis]
                # Note: the following still includes hip joints; this will be removed soon:
                full_poses.append(datum.poseKeypoints[:, [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 12, 15, 16, 17, 18], :])

        print('Finished recording.')
        self.video_device.release() 

        full_pose_array = np.vstack(full_poses)
        if save_data:
            print('Saving recorded pose data to npy binary files...')
            if filename is None:
                filename = 'saved_gesture_pose_data'
            np.save(os.path.join('data/gesture_examples/', filename), full_pose_array)

        return full_pose_array

    def classify_gesture(self, sample_pose_array):
	sample_pose_array = np.delete(sample_pose_array, [8, 9, 10], axis=1)
	sample_pose_array = self._preprocess_data(sample_pose_array)
	sample_most_varying_indices = self._get_n_most_varying_dim_indices(sample_pose_array)

	most_likely_class, min_distance = self._get_nearest_neighbour_class(sample_pose_array, sample_most_varying_indices)

        if self.debug:
            print(self.gesture_distances_dict)

        if self.gesture_distance_threshold is not None:
            if min_distance < self.gesture_distance_threshold:
                print('This is most likely a {} gesture'.format(most_likely_class))
            else:
                print('Gesture not recognized!')
        else:
            print('This is most likely a {} gesture'.format(most_likely_class))

	return most_likely_class

    def _load_known_example_data(self):
	filenames = [filename for filename in os.listdir(self.known_example_data_dir) if filename.endswith('npy')]

        for filename in filenames:
            for gesture_type in self.gesture_example_data_dict.keys():
                if gesture_type in filename:
                    break
            else:
                print('WARNING: Loaded file does not match any known gesture type!!')
            pose_array = self._preprocess_data(np.load(os.path.join(self.known_example_data_dir, filename)))
            pose_array = np.delete(pose_array, [8, 9, 10], axis=1)
            self.gesture_example_data_dict[gesture_type].append(pose_array)
            self.gesture_example_pc_indices_dict[gesture_type].append(self._get_n_most_varying_dim_indices(pose_array))

    def _get_nearest_neighbour_class(self, sample_data, sample_most_varying_indices):
	min_distance = np.inf
	most_likely_class = None

	for gesture_type in self.gesture_example_data_dict.keys():
	    for example_num in range(len(self.gesture_example_data_dict[gesture_type])):
		distance = self._get_gesture_distance(sample_data,
				                      self.gesture_example_data_dict[gesture_type][example_num],
					              sample_most_varying_indices,
					              self.gesture_example_pc_indices_dict[gesture_type][example_num])
		self.gesture_distances_dict[gesture_type].append(distance)

		if distance < min_distance:
		    min_distance = distance
		    most_likely_class = gesture_type
 
	return most_likely_class, min_distance

    def _perform_shoulder_scaling(self, pose_array):
	abs_shoulder_distances = np.linalg.norm(pose_array[:, 2, :] - pose_array[:, 5, :], axis=1)
	return pose_array / abs_shoulder_distances[np.newaxis][np.newaxis].T

    def _perform_neck_scaling(self, pose_array):
	neck_array = pose_array[:, 1, :].reshape(pose_array[:, 1, :].shape[0], 1, pose_array[:, 1, :].shape[1])
	return pose_array - neck_array

    def _normalize_mean(self, pose_array):
	return pose_array - np.mean(pose_array, axis=0)[np.newaxis]

    def _preprocess_data(self, pose_array):
	pose_array = self._perform_neck_scaling(pose_array)
	pose_array = self._perform_shoulder_scaling(pose_array)
	pose_array = median_filter(pose_array, size=2)
	pose_array = self._normalize_mean(pose_array)
	pose_array = gaussian_filter1d(pose_array, axis=0, sigma=1)
	
	return pose_array

    def _get_n_most_varying_dim_indices(self, pose_array, n=5):
	list_of_indices = []
	for spatial_dim in range(pose_array.shape[2]):
	    most_varying_indices = np.flip(np.argsort(np.var(pose_array, axis=0)[:, spatial_dim]))[:n]
	    list_of_indices.append(set(most_varying_indices))
	    
	return list_of_indices

    def _get_gesture_distance(self, pose_array_1, pose_array_2, array_1_indices, array_2_indices):
	distances = []
	for dim in range(3):
	    dist, _ = fastdtw(pose_array_1[:, list(array_1_indices[dim] | array_2_indices[dim]), dim], 
			      pose_array_2[:, list(array_1_indices[dim] | array_2_indices[dim]), dim], 
			      dist=euclidean)
	    distances.append(dist)

	return np.linalg.norm(distances)

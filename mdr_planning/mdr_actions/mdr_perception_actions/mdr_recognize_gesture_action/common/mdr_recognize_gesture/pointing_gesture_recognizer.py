#!/usr/bin/env python

from __future__ import print_function
import sys
import operator

import cv2
import numpy as np
from openpose import pyopenpose as op


class PointingGestureRecognizer(object):
    def __init__(self, obj_distance_to_line_threshold, model_path, debug):
        self.obj_distance_to_line_threshold = obj_distance_to_line_threshold
        self.debug = debug

        self.opWrapper = op.WrapperPython()
        self.opWrapper.configure({"model_folder" : model_path})
        self.opWrapper.start()

    def get_object_pointed_to(self, bbs, poses, labels, frame):
        success = False

        datum = op.Datum()
        datum.cvInputData = frame
        self.opWrapper.emplaceAndPop([datum])
  
        if datum.poseKeypoints.shape == ():
            if self.debug:
                print('Body probably too close to adequately estimate pose. Ignoring...')
            return success, None, frame

        neck_pose = datum.poseKeypoints[0, 1, :]
        left_hand_pose = datum.poseKeypoints[0, 7, :]
        left_elbow_pose = datum.poseKeypoints[0, 6, :]
        right_hand_pose = datum.poseKeypoints[0, 4, :]
        right_elbow_pose = datum.poseKeypoints[0, 3, :]

        right_arm_visible = right_elbow_pose[2] > 0.1 and right_hand_pose[2] > 0.1
        left_arm_visible = left_elbow_pose[2] > 0.1 and left_hand_pose[2] > 0.1

        if right_arm_visible or left_arm_visible:
            objs_nearest_to_lines_indices = [None, None]
            obj_distance_to_lines = [None, None]

            if right_arm_visible:
                obj_index_right, obj_distance_right, frame = self.process_arm_pose(right_elbow_pose, right_hand_pose,
                                                                                   neck_pose, bbs, poses, labels, frame)
                objs_nearest_to_lines_indices[0] = obj_index_right
                obj_distance_to_lines[0] = obj_distance_right
                success = True

            if left_arm_visible:
                obj_index_left, obj_distance_left, frame = self.process_arm_pose(left_elbow_pose, left_hand_pose,
                                                                                 neck_pose, bbs, poses, labels, frame)
                objs_nearest_to_lines_indices[1] = obj_index_left
                obj_distance_to_lines[1] = obj_distance_left
                success = True

            if any(obj_distance_to_lines):
                min_arg = obj_distance_to_lines.index(min([x for x in obj_distance_to_lines if x is not None]))
                object_index = objs_nearest_to_lines_indices[min_arg]
                if self.debug: 
                    print('Pointing to object:', object_index)
                frame = cv2.rectangle(frame, (bbs[object_index][0] - 5 , bbs[object_index][1] - 5), 
                                      (bbs[object_index][0] + bbs[object_index][2] + 5 , bbs[object_index][1] + bbs[object_index][3] + 5), 
                                      (0, 0, 255), 3)
                frame = cv2.putText(frame, 'Pointing to {}'.format(labels[object_index]), (50,50), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (0, 0, 255), 2, cv2.LINE_AA) 

                return success, object_index, frame

        return success, None, frame
    
    def process_arm_pose(self, elbow_pose, hand_pose, neck_pose, bbs, poses, labels, frame, use_pose_estimates_for_decision=True):
        obj_nearest_to_line_index = None
        distance_to_hand = None
        center_distances_to_line = []
        nearest_points_to_line = []

        bb_centers, frame = self.get_bb_centers(bbs, frame, False) 

        line_params, frame, line_points = self.get_pointing_line(elbow_pose, hand_pose, frame, False)

        # Find the point on the line nearest to each bb center, and the perpendicular distances between them.
        for i in range(len(bbs)):
            dist, nearest_point = self.get_shortest_distance_to_segment(bb_centers[i], line_points)
            nearest_points_to_line.append(nearest_point)

            if labels[i] == 'person' and self.point_in_bb(bbs[i], neck_pose):
                # Eliminate pointing person from consideration by setting a high distance value
                center_distances_to_line.append(float('inf'))
            elif self.point_in_bb(bbs[i], hand_pose) or self.point_in_bb(bbs[i], elbow_pose):
                # Similarly eliminate objects if hand or elbow are within them
                center_distances_to_line.append(float('inf'))
            else:
                center_distances_to_line.append(dist)

        # Find hypothesis set of objects; those whose distances to the pointing line fall under a given threshold:
        hypothesis_obj_indices = np.where(np.array(center_distances_to_line) < self.obj_distance_to_line_threshold)[0]

        if hypothesis_obj_indices.size != 0:
            if hypothesis_obj_indices.size == 1:
                obj_nearest_to_line_index = int(hypothesis_obj_indices)
                distance_to_hand = np.linalg.norm(bb_centers[obj_nearest_to_line_index] - hand_pose[0:2])
            else:
            # If multiple objects, take the one closest to the pointing hand:
                print('Possible objects:', [labels[i] for i in hypothesis_obj_indices])
                if use_pose_estimates_for_decision:
                    # Utilize pose info to find object whose pose is closest to hand's:
                    object_pose_array = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in poses])[hypothesis_obj_indices]
                    distances_to_hand = np.linalg.norm(object_pose_array - hand_pose, axis=1)
                else:
                    # Use naive rule: distances of bb centers to hand position
                    distances_to_hand = np.linalg.norm(np.array(bb_centers)[hypothesis_obj_indices] - hand_pose[0:2], axis=1)
                obj_nearest_to_line_index = hypothesis_obj_indices[np.argmin(distances_to_hand)]
                distance_to_hand = np.min(distances_to_hand)

        return obj_nearest_to_line_index, distance_to_hand, frame

    def get_shortest_distance_to_segment(self, point, line_points):
        '''Inspired by comment in following topic:
        https://stackoverflow.com/a/51240898
        '''
        line_vec = (line_points[2] - line_points[0],
                    line_points[3] - line_points[1])
        point_vec = (point[0] - line_points[0],
                     point[1] - line_points[1])
        line_len = np.sqrt(line_vec[0]**2 + line_vec[1]**2)
        unit_line_vec = (line_vec[0] / line_len, line_vec[1] / line_len)
        point_vec_scaled = (point_vec[0] * (1./line_len), point_vec[1] * (1./line_len))
        t = np.array(unit_line_vec).dot(np.array(point_vec_scaled).T)
        if t < 0.0:
            t = 0.0
        elif t > 1.0:
            t = 1.0
        nearest_point = (line_vec[0] * t, line_vec[1] * t)
        dist = np.sqrt((nearest_point[0] - point_vec[0])**2 + (nearest_point[1] - point_vec[1])**2)
        nearest_point = tuple(map(operator.add, nearest_point, line_points[0:2]))
        return (dist, nearest_point)

    def get_line_params(self, point_1, point_2):
        '''Inspired by comment in following topic:
        https://stackoverflow.com/a/13242831
        '''
        a = point_1[1] - point_2[1]
        b = point_2[0] - point_1[0]
        c = ((point_1[0] - point_2[0]) * point_1[1]) + ((point_2[1] - point_1[1]) * point_1[0])
        return float(a), float(b), float(c)

    def get_pointing_line(self, elbow_pose, hand_pose, image, draw_on_image=False):
        width = image.shape[1]
        a, b, c = self.get_line_params(elbow_pose[0:2], hand_pose[0:2])
        if hand_pose[0] < elbow_pose[0]:
            end_point = [0, 0]
        else:
            end_point = [int(width), 0]
        end_point[1] = int(-((a/b) * end_point[0]) - (c/b))
        if draw_on_image:
            image = cv2.line(image, tuple(elbow_pose[0:2]), tuple(end_point), 
                             (0, 0, 255), thickness=3)
        return (a, b, c), image, (hand_pose[0], hand_pose[1], end_point[0], end_point[1]) 

    def get_bb_centers(self, bbs, frame, draw_on_image=True):
        bb_centers = []
        for bb in bbs:
            length_x = bb[2]
            length_y = bb[3]
            bb_centers.append(((bb[0] + int(0.5 * length_x)), (bb[1] + int(0.5 * length_y))))

            if draw_on_image:
                frame = cv2.circle(frame, bb_centers[-1], 1, (0, 255, 0), 3)
        return bb_centers, frame

    def point_in_bb(self, bb, point):
        return point[0] > bb[0] and point[0] < (bb[0] + bb[2]) and point[1] > bb[1] and point[1] < (bb[1] + bb[3])

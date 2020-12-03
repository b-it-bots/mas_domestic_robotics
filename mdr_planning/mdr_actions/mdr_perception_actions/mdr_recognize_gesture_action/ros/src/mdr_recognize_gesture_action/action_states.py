from __future__ import print_function
from importlib import import_module
import copy

import numpy as np
import cv2

import rospy
import actionlib
import torch
import torchvision
from cv_bridge import CvBridge

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_recognize_gesture.pointing_gesture_recognizer import PointingGestureRecognizer
from mdr_recognize_gesture.gesture_classifier import GestureClassifier

from mas_perception_libs.utils import cloud_msg_to_image_msg, cloud_msg_to_cv_image
import dataset_interface.object_detection.transforms as T
from mas_tools.ros_utils import get_package_path

from sensor_msgs.msg import PointCloud2, Image
from mas_perception_msgs.msg import DetectObjectsAction, DetectObjectsGoal
from mdr_recognize_gesture_action.msg import RecognizeGestureGoal, RecognizeGestureResult


class RecognizeGestureSM(ActionSMBase):
    cv_bridge_client = None
    pytorch_detector = None
    gesture_classifier = None
    pointing_gesture_recognizer = None
    image_pub = None
    def __init__(self, timeout=120.0,
                 gesture_type=None,
                 openpose_models_dir='/home/lucy/.models/openpose_models/',
                 pointcloud_topic='/mdr_perception/rectified_points',
		 object_detection_server='/object_detection_server',
                 max_recovery_attempts=1):
        super(RecognizeGestureSM, self).__init__('RecognizeGesture', [], max_recovery_attempts)

        self.timeout = timeout
        self.gesture_type = gesture_type
        self.openpose_models_dir = openpose_models_dir
        self.object_detection_server = object_detection_server
        self.pointcloud_topic = pointcloud_topic

    def init(self):
        self.cv_bridge_client = CvBridge()
        self.pytorch_detector = torchvision.models.detection.fasterrcnn_resnet50_fpn(pretrained=True)
        self.pytorch_detector.eval()
        torch_device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        self.pytorch_detector.to(torch_device)
        self.gesture_classifier = GestureClassifier(gesture_distance_threshold=50.,
                                                    model_path=self.openpose_models_dir,
                                                    gesture_types=['waving', 'nodding', 'shaking_head', 'go_away', 'come_closer', 'pointing'],
                                                    known_example_data_dir=get_package_path('mdr_recognize_gesture_action', 'data', 'gesture_examples'))
        self.pointing_gesture_recognizer = PointingGestureRecognizer(60., self.openpose_models_dir, False)

        self.image_pub = rospy.Publisher("pointing_gesture_recognizer_image", Image, queue_size=1)

        try:
            self.object_detection_client = actionlib.SimpleActionClient(self.object_detection_server, DetectObjectsAction)
            rospy.loginfo('[recognize_gesture] Waiting for %s server', self.object_detection_server)
            print(self.object_detection_server)
            self.object_detection_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[recognize_gesture] %s', str(exc))
            return FTSMTransitions.INIT_FAILED

        return FTSMTransitions.INITIALISED

    def _draw_bbs(self, bbs, frame):
        for bb in bbs:
            frame = cv2.rectangle(frame, (bb[0], bb[1]), (bb[0]+bb[2], bb[1]+bb[3]), (0, 255, 0), 3)
        return frame

    def _convert_bb_format(self, bbs):
        '''Convert bounding boxes from list of sensor_msgs.RegionOfInterest format
        to a numpy array of shape (num_bbs, 4), where each row contains (x, y, w, h) of the respective bb.
        '''
        converted_bbs = np.zeros((len(bbs), 4)).astype(int)
        for i in range(len(bbs)):
            converted_bbs[i, 0] = bbs[i].x_offset
            converted_bbs[i, 1] = bbs[i].y_offset
            converted_bbs[i, 2] = bbs[i].width
            converted_bbs[i, 3] = bbs[i].height
        return converted_bbs

    def _get_object_info(self, object_list):
        bbs, poses, labels = [], [], []
	for obj in object_list:
            bbs.append(obj.roi)
            poses.append(obj.pose.pose)
            labels.append(obj.category)
        return bbs, poses, labels

    def _filter_out_bbs_by_distance(self, bbs, poses, labels=None, threshold=5.):
        filtered_object_indices = []
        for i in range(len(poses)):
            if poses[i].position.x > threshold:
                filtered_object_indices.append(i)
                if labels is not None:
                    print('Filtering out object: {}'.format(labels[i]))
        bbs = np.delete(bbs, filtered_object_indices, axis=0)
        labels = np.delete(np.array(labels), filtered_object_indices).tolist()
        return bbs, labels

    def _print_object_data(self, bbs, poses, labels):
        print('Printing object information:')
        for i in range(bbs.shape[0]):
            print('Object: {}'.format(labels[i]))
            print('Bounding box:\n', bbs[i])
            print('Pose:\n',poses[i])
            print()

    def running(self):
        rospy.loginfo('[recognize_gesture] Recognizing gesture')
        self.object_detection_client.send_goal(DetectObjectsGoal())
        self.object_detection_client.wait_for_result(timeout=rospy.Duration.from_sec(50.))
        detection_result = self.object_detection_client.get_result()

        cv_img = self.cv_bridge_client.imgmsg_to_cv2(detection_result.image, desired_encoding="passthrough")

        if self.gesture_type is None:
            rospy.loginfo('[recognize_gesture] Classifying gesture...')
            gesture_data = gesture_classifier.capture_gesture_data()
            self.gesture_type = gesture_classifier.classify_gesture(gesture_data)

            if self.gesture_type is None:
                rospy.loginfo('[recognize_gesture] Gesture not recognized!')
                self.result = self.set_result(False)
                return FTSMTransitions.DONE
            else:
                rospy.loginfo('This is most likely a {} gesture'.format(self.gesture_type))

        if self.gesture_type == 'pointing':
            rospy.loginfo('[recognize_gesture] Processing pointing gesture')

            bbs, poses, labels = self._get_object_info(detection_result.objects.objects)
            bbs = self._convert_bb_format(bbs)

            bbs, labels = self._filter_out_bbs_by_distance(bbs, poses, labels, 5.)

            success, obj_index, frame = self.pointing_gesture_recognizer.get_object_pointed_to(bbs, poses, labels, cv_img)
            # For visualization:
            # cv_img = self._draw_bbs(bbs, cv_img)
            # self.image_pub.publish(self.cv_bridge_client.cv2_to_imgmsg(cv_img, "bgr8"))

            print('Object pointed to:', labels[obj_index] if obj_index is not None else str(None))
            self.result = self.set_result(True)

        return FTSMTransitions.DONE

    def recovering(self):
        ## TODO: implement any recovery behaviours here
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = RecognizeGestureResult()
        result.success = success
        return result

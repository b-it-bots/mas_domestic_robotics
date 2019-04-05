#!/usr/bin/python
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_detect_person.msg import DetectPersonFeedback, DetectPersonGoal, DetectPersonResult
from mdr_perception_msgs.msg import FaceBoundingBox
from mdr_detect_person.inference import load_detection_model, detect_faces

class DetectPersonSM(ActionSMBase):
    def __init__(self, timeout=120.,
                 image_topic='/cam3d/rgb/image_raw',
                 detection_model_path='',
                 max_recovery_attempts=1):
        super(DetectPersonSM, self).__init__('DetectPerson', [], max_recovery_attempts)
        self.timeout = timeout
        self.detection_model_path = detection_model_path
        self.bridge = CvBridge()
        self.image_publisher = rospy.Publisher(image_topic, Image, queue_size=1)
        self.face_detection = None

    def init(self):
        try:
            rospy.loginfo('[detect_person] Loading detection model %s', self.detection_model_path)
            self.face_detection = load_detection_model(detection_model_path)
        except:
            rospy.logerr('[detect_person] Model %s could not be loaded', self.detection_model_path)
        return FTSMTransitions.INITIALISED

    def running(self):
        bounding_boxes = []
        number_of_faces = 0
        detection_successful = True
        input_image = self.__convert_image(self.goal.image)
        try:
            bgr_image = input_image
            gray_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
            rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
            faces = detect_faces(self.face_detection, gray_image)
            number_of_faces = np.size(faces, 0)

            for face_coordinates in faces:
                bounding_box = FaceBoundingBox()
                bounding_box.bounding_box_coordinates = face_coordinates.tolist()
                bounding_boxes.append(bounding_box)

                x, y, w, h = face_coordinates
                rgb_cv2 = cv2.rectangle(rgb_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            output_ros_image = self.bridge.cv2_to_imgmsg(rgb_image, 'bgr8')
            self.image_publisher.publish(output_ros_image)
        except:
            detection_successful = False

        self.result = self.set_result(detection_successful, number_of_faces, bounding_boxes)
        return FTSMTransitions.DONE

    def __convert_image(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        return np.array(cv_image, dtype=np.uint8)

    def set_result(self, detection_successful, number_of_faces, bounding_boxes):
        result = DetectPersonResult()
        result.success = detection_successful
        result.number_of_faces = number_of_faces
        result.bounding_boxes = bounding_boxes
        return result

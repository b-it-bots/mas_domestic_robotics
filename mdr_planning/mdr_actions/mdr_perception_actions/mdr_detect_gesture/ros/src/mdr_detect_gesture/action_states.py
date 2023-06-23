#!/usr/bin/python
import rospy
import cv2
import numpy as np
import time
from mdr_detect_gesture.inference import MultiPerDetector
from mdr_detect_gesture.person_det_media import Person_Detector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_detect_gesture.msg import DetectGestureResult, DetectGestureActionResult
from mdr_perception_msgs.msg import BodyBoundingBox
from tensorflow.keras.models import load_model
from collections import deque


class DetectGestureSM(ActionSMBase):
    def __init__(self, timeout=120.,
                 gesture_topic = 'heartmet/gesture',
                 detection_model_path='',   
                 max_recovery_attempts=1):
        super(DetectGestureSM, self).__init__('DetectGesture', [], max_recovery_attempts)
        self.timeout = timeout
        self.detection_model_path = detection_model_path
        self.bridge = CvBridge()
       
        self.model = None
        self.gesture_detection = None
        self.num_person = 1
        self.rp = Person_Detector()
        #self.image = Image()
        self.input_image = np.zeros((480,640, 3),dtype=np.uint8)
        self.gest_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.image_callback)
        self.d = deque(maxlen=60)
        self.seq_len=10
        self.keys = deque(maxlen=self.seq_len)
        self.avg_len = 10
        self.frames = deque(maxlen=self.avg_len)




    def init(self):
        try:
            rospy.loginfo('[detect_person] Loading detection model %s', self.detection_model_path)
            self.model = MultiPerDetector(self.rp, num_person = self.num_person, hand_model='/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_actions/mdr_perception_actions/mdr_detect_gesture/model/action8_hand2d.tflite', head_model = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_actions/mdr_perception_actions/mdr_detect_gesture/model/action3_head2d_rot.tflite')
        except Exception as exc:
            rospy.logerr('[detect_gesture] Model %s could not be loaded: %s',
                         self.detection_model_path, str(exc))
        return FTSMTransitions.INITIALISED

    def image_callback(self,msg):
        # print(msg)
        #self.image = msg
        # self.input_image = self.__convert_image(msg)
        cv_image = self.bridge.imgmsg_to_cv2(msg)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        self.input_image = cv_image

    def image_process(self):

        try:
            bgr_image = self.input_image.copy()
            #print("image", bgr_image)
            # for i in range(self.num_person):
                # self.model.globals()["per"+str(i)].keys= deque(maxlen=10)
            self.frames.append(bgr_image)
            if len(self.frames) == 10:
                image, gestures = self.model.multi_per_gesture(bgr_image,sort="area",viz=False)
                print(gestures)
                avggestures = self.model.avg_per_gesture()
                # print("gestures", avggestures)
                for gest in avggestures:

                    gesture_msg = DetectGestureResult()
                    gesture_msg.gesture = gest[0]
                    gesture_msg.gesture_selection = gest[4]
                    gesture_msg.confidence = gest[1]
                    gesture_msg.id = gest[2]
                    gesture_msg.bounding_box.bounding_box_coordinates = gest[3]
    
                    self.result = gesture_msg

                    self.frames = deque(maxlen=self.avg_len)


        except Exception as e:
            detection_successful = False
            print(e)

    def running(self):

        # looprate = rospy.Rate(10)
        
        while self.result == None:
            rospy.sleep(0.07)
            self.image_process()
            # print("while I am in running and result is none")
        #     #looprate.sleep()
        #self.gest_sub.unregister()
        
        return FTSMTransitions.DONE

    def __convert_image(self, ros_image):

        cv_image = self.bridge.imgmsg_to_cv2(ros_image, 'bgr8')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        return cv_image
    

    def set_result(self, detection_successful, gesture, confidence, bounding_box, id):
        result = DetectGestureResult()
        result.success = detection_successful
        result.gesture = gesture
        result.confidence = confidence
        result.id = id
        result.bounding_box = bounding_box
        return result
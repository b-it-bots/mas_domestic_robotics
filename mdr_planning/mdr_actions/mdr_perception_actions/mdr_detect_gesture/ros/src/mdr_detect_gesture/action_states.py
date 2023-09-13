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
        self.num_person = 2
        self.rp = Person_Detector()
        #self.image = Image()
        self.input_image = np.zeros((480,640, 3),dtype=np.uint8)
        self.gest_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.image_callback)


    def init(self):
        model_path = "/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_actions/mdr_perception_actions/mdr_detect_gesture/model/"
        try:
            rospy.loginfo('[detect_person] Loading detection model %s', self.detection_model_path)
            self.model = MultiPerDetector(self.rp, num_person = self.num_person, 
                                          hand_model=model_path+'action8_hand2d_10f_2.h5', 
                                          head_model=model_path+'action3_head2d_rot_10fabs.h5',
                                          fing_model=model_path+"gest_num_classifier.h5")
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
            image, gestures = self.model.multi_per_gesture(bgr_image,viz=True)
            #cv2.imshow("hdhd",image)
            #cv2.waitKey(1)
            # rospy.loginfo("         ")
            # rospy.loginfo("         ")
            # rospy.loginfo("         ")
            # rospy.loginfo(gestures)
            # rospy.loginfo("         ")
            # rospy.loginfo("         ")
            # rospy.loginfo("         ")
            found = False
            for i in range(self.num_person):
                if gestures[i]!=[]:
                    found = True
            if found:
                # rospy.loginfo("         ")
                # rospy.loginfo("         ")
                # rospy.loginfo("         ")
                # rospy.loginfo("--------------------gesture found----------------------")
                # rospy.loginfo("         ")
                # rospy.loginfo("         ")
                # rospy.loginfo("         ")
                comb_gesture = self.model.combine_multi_per_gest(gestures)
                # print("gestures", avggestures)
                gesture_msg = DetectGestureResult()
                gesture_msg.gesture = comb_gesture[0]
                gesture_msg.gesture_selection = comb_gesture[4]
                gesture_msg.confidence = comb_gesture[1]
                gesture_msg.id = comb_gesture[2]
                gesture_msg.bounding_box.bounding_box_coordinates = comb_gesture[3]
                print(gesture_msg)
                self.result = gesture_msg
        except Exception as e:
            detection_successful = False
            print(e)

    def running(self):

        # looprate = rospy.Rate(10)
        
        while self.result == None:
            self.image_process()
        #cv2.destroyAllWindows
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

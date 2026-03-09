"""
Receptionist Challenge - 2026
Authors: Khawaja Saad
saadbutt15@outlook.com
"""
import rospy
import time
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mas_hsr_head_controller.head_controller import HeadController
from mdr_composite_behaviours.coordetector import t2d2t3d
import torch
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
import pandas as pd
from mdr_composite_behaviours.Nav_Man import Mover
from geometry_msgs.msg import PoseStamped, PoseArray
from ultralytics import YOLO  # Updated for YOLOv26
import moveit_commander

class ReceptionistTask(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'person_pose_detect',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['person_pose'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)
        self.person_data={'person_pose':None} ## list of ndarray
        self.bridge = CvBridge()
        
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.callback)
        self.cloud_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, self.callback1)
        self.annotated_image = None
        
        self.image_pub = rospy.Publisher("image_topic", Image, queue_size=10)
        self.listening_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Listening_Image.jpg'
        self.processing_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Processing_Image.jpg'
        self.error_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Error_Image.jpg'
        self.idle_img_path = '/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/ros/src/mdr_hri_behaviours/Idle_Ellipsis_Image.jpg' 
        
        # Updated to YOLOv26
        self.model = YOLO('yolo26n.pt') 
        self.td23D = t2d2t3d()
        rospy.logerr('Initialising moveit group')
        self.head = moveit_commander.MoveGroupCommander("head")
        rospy.logerr('Initialised moveit group')

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        self.say(text)
        
    def update_display_with_image(self, display_type):
        if display_type == 'listening':
            img_path = self.listening_img_path
        elif display_type == 'processing':
            img_path = self.processing_img_path
        elif display_type == 'idle':
            img_path = self.idle_img_path
        elif display_type == 'error':
            img_path = self.error_img_path
        else:
            rospy.loginfo("Unknow display type: {}".format(display_type))
        img = cv2.imread(img_path)
        image_message =self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        self.image_pub.publish(image_message)

    def callback1(self,data):
        try:
            self.cloud_data = data
        except ValueError as e:
            print(e)

    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
            print(e)

    def execute(self, userdata):
        self.tilt_angle = 0.0
        self.head.set_joint_value_target("head_tilt_joint", self.tilt_angle)
        self.head.go()
        
        rospy.loginfo("Initiating receptionist interaction...")
        rospy.sleep(2) 
                
        # YOLOv26 Inference - filtering for class 0 (person)
        results = self.model.predict(self.cv_image, classes=[0], conf=0.5)
        
        detected = False
        box = None
        
        # YOLOv26 returns a list of Results objects
        for result in results:
            if len(result.boxes) > 0:
                # Get the first detected person
                b = result.boxes[0].xyxy[0].cpu().numpy() 
                box = [[int(b[0]), int(b[1])], [int(b[2]), int(b[3])]]
                detected = True
                break

        if not detected:
            rospy.logwarn("No person detected.")
            return 'failed'

        print("Detection DONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        whole, obj_clus = self.td23D.get_box_voxel(box, self.cloud_data)
        obj_pose = self.td23D.get_3D_cords(obj_clus)
        print(obj_pose)
        
        self.person_data["person_pose"] = obj_pose
        userdata.person_pose = [self.person_data]

        return 'succeeded'
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
from mdr_perception_msgs.msg import BodyBoundingBox
#from mdr_percieve_plane_actions.action_states import PerceivePlaneSM


class DetectDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'detect_lever',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['lever_pose'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'detect_lever')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.retry_count = 0
        self.timeout = 120.
        self.head= HeadController()
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.callback)
        self.cloud_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", PointCloud2, self.callback1)
        self.lever_bbox_sub = rospy.Subscriber("/heartmet/detection_bbox_door", BodyBoundingBox, self.lever_bbox_callback)
        
        self.mover = Mover()
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_composite_behaviours/ros/models/erl_door.pt')
        #self.td23D = t2d2t3d()
        #ppsm = PerceivePlaneSM()
        #self.model = ppsm.model

    def execute(self, userdata):
        rospy.loginfo('[detect_lever] Trying to detect nearest lever')

        self.say('In state detect lever')
        self.say('Trying to detect nearest lever')  

        result = self.model(self.cv_image, size=416)
        print(result.pandas().xyxy[0])
        df = result.pandas().xyxy[0]
        detected = False
        #result = df[df['class'] == 2][['xmin', 'ymin', 'xmax', 'ymax']]
        for index, row in df.iterrows():
            if row['class'] == 2:
                  detected= True
                  box = [[int(row['xmin']),int(row['ymin'])],[int(row['xmax']),int(row['ymax'])]]
        #print(f"Row {index}: xmin={xmin}, ymin={ymin}, xmax={xmax}, ymax={ymax}")
        if not detected:
            return 'failed'
        print("Detection DONE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")
        cloud = self.cloud_data
        whole, obj_clus = self.td23D.get_box_voxel(box, cloud)
        obj_pose = self.td23D.get_3D_cords(obj_clus)
        print(obj_pose)
        

        real_object_head = self.mover.transform_3D2head([obj_pose])[0]
        #real_object_pose = self.mover.transform_head2map([real_object_head])[0]
        real_object_pose = self.mover.transform_head2base([real_object_head])[0] 
        # pose_ = PoseStamped()
        # real_object_pose
        # x = pose_.pose.position.x
        # y = pose_.pose.position.y
        # z = pose_.pose.position.z
        # ox = pose_.pose.orientation.x
        # oy = pose_.pose.orientation.y
        # oz = pose_.pose.orientation.z
        # ow = pose_.pose.orientation.w
        lever_pose=[real_object_pose.pose.position.x, real_object_pose.pose.position.y, real_object_pose.pose.position.z, real_object_pose.pose.orientation.x,real_object_pose.pose.orientation.y,real_object_pose.pose.orientation.z,real_object_pose.pose.orientation.w]          
        # lever_pose={'x':real_object_pose.pose.position.x; 'y':real_object_pose.pose.position.y; 'z':real_object_pose.pose.position.z;  'ox':real_object_pose.pose.orientation.x; 'oy':real_object_pose.pose.orientation.y; 'oz':real_object_pose.pose.orientation.z ; 'ow': real_object_pose.pose.orientation.w }
        userdata.lever_pose=lever_pose
        print(real_object_head)
        print(real_object_pose)

        val1=self.head.turn_left()
        val=self.head.turn_right()
        rospy.loginfo('[detect_lever] Trying to detect nearest lever')        
        if val == True:
            self.say('I turned my head to left')
        else:
            self.say('Turning the head to right')


        return 'succeeded'

    def lever_bbox_callback(self, data):
        try:
            self.lever_box_data = data
        except ValueError as e:
            print(e)
        

    def callback1(self,data):
        #print('callback')
        try:
            self.cloud_data = data
            #print(self.cv_image)
        except ValueError as e:
            print(e)

    def callback(self,data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data)
            #self.cv_image = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
        except CvBridgeError as e:
            self.cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
            print(e)

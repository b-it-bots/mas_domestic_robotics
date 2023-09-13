import rospy
import time
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mas_hsr_head_controller.head_controller import HeadController
from mdr_composite_behaviours.coordetector import t2d2t3d
import torch
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2
import pandas as pd
from mdr_composite_behaviours.Nav_Man import Mover
from mdr_composite_behaviours.force_feedback import ForceToVelocityNode



class DetectDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'detect_lever',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['wrist_direction'])
        
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'detect_lever')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.retry_count = 0
        self.timeout = 120.
        self.head= HeadController()
        self.bridge = CvBridge()
       
        self.forceVel=None
        # self.wrist_direction = userdata.wrist_direction
        # self.wrist_direction = list(kwargs.get('wrist_direction', list()))
        # self.forceVel=ForceToVelocityNode(self.door_direction)
        # self.image_sub = rospy.Subscriber("/hsrb/head_rgbd_sensor/rgb/image_raw", Image, self.callback)
        

    def execute(self, userdata):
        rospy.loginfo('[detect_lever] Trying to detect nearest lever')

                
        if len(self.lever_pose) == 0:
            self.wrist_direction = userdata.wrist_direction
            rospy.loginfo("Using userdata's door_direction {0}".format(self.door_direction))

        self.say('In state door opening')
        self.say('Trying to detect nearest lever')  
        
        self.forceVel=ForceToVelocityNode(self.door_direction)
        self.forceVel.run()

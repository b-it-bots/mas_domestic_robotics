import rospy
import time
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mas_hsr_head_controller.head_controller import HeadController
import torch

class DetectDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'detect_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'detect_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.retry_count = 0
        self.timeout = 120.
        self.head= HeadController()
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_composite_behaviours/ros/models/erl_door.pt')

    def execute(self, userdata):
        rospy.loginfo('[detect_door] Trying to detect nearest door')

        self.say('In state detect door')
        self.say('Trying to detect nearest door')  
        self.say('Turning the head to left')
        val1=self.head.turn_left()
        val=self.head.turn_right()
        rospy.loginfo('[detect_door] Trying to detect nearest door')        
        if val == True:
            self.say('I turned my head to left')
        else:
            self.say('Turning the head to right')


        return 'succeeded'
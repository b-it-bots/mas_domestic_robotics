#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import Image
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_base_planner.msg import BasePlannerAction, BasePlannerGoal


class InterationClient(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'InteractionClient',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['command'])
        
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 120.)
        self.client = actionlib.SimpleActionClient('mdr_actions/detect_gesture_server', BasePlannerAction)
        self.threshold = kwargs.get('threshold', 0.68)
        self.goal = BasePlannerGoal()
        self.client.wait_for_server()

 
    def execute(self, userdata):
        
        rospy.loginfo('Starting Interaction module ...')
        
        self.goal.start = True

        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        print(self.client.get_result())
        # self.say("I could not hear you, can you please respond via gesture")
        if self.client.get_result():
            return 'succeeded'
        return 'failed'


ob = InterationClient()
ob.execute()
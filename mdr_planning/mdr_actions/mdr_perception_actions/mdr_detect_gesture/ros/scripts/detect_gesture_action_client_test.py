#!/usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import Image
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_detect_gesture.msg import DetectGestureAction, DetectGestureGoal, DetectGestureFeedback, DetectGestureResult


class InterationClient(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'InteractionClient',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['command'])
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 120.)

        self.threshold = kwargs.get('threshold', 0.68)

 
    def execute(self, userdata):
        
        rospy.loginfo('Starting Interaction module ...')

        client = actionlib.SimpleActionClient('mdr_actions/detect_gesture_server', DetectGestureAction)
        client.wait_for_server()

        goal = DetectGestureGoal()
        
        goal.start = False

        client.send_goal(goal)
        client.wait_for_result()
        print(client.get_result())
        print("sleeping for 10 seconds")
        # rospy.sleep(10)
        # goal.start = False
        # client.send_goal(goal)
        # client.wait_for_result()
        print(client.get_result())
        self.say("I could not hear you, can you please respond via gesture")
        if not client.get_result:
            return 'succeeded'
        return 'failed'


# ob = InterationClient()
# ob.execute()
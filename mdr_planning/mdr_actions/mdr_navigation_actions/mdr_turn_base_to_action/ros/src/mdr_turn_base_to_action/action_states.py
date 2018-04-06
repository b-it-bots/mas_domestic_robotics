#!/usr/bin/python

import rospy
import smach
import smach_ros
from actionlib import SimpleActionClient


from geometry_msgs.msg import Quaternion
from mdr_turn_base_to_action.msg import TurnBaseToFeedback, TurnBaseToResult


class SetupTurnBaseTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['turn_to_feedback'])

    def execute(self, userdata):
        feedback = TurnBaseToFeedback()
        feedback.message = '[enter_door] entering door'
        userdata.enter_door_feedback = feedback

        return 'succeeded'


class TurnBaseTo(smach.State):
    def __init__(self, timeout=120.0,
                 move_forward_server='/mdr_actions/move_forward_server',
                 movement_duration=15., speed=0.1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.movement_duration = movement_duration
        self.speed = speed

        self.move_forward_client = SimpleActionClient(move_forward_server,
                                                      MoveForwardAction)
        self.move_forward_client.wait_for_server()

        self.entered = False

    def execute(self, userdata):
        goal = MoveForwardGoal()
        goal.movement_duration = self.movement_duration
        goal.speed = self.speed

        self.move_forward_client.send_goal(goal)
        timeout_duration = rospy.Duration.from_sec(self.timeout)
        self.move_forward_client.wait_for_result(timeout_duration)

        result = self.move_forward_client.get_result()
        if result and result.success:
            return 'succeeded'
        else:
            return 'failed'


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['enter_door_result'])
        self.result = result

    def execute(self, userdata):
        result = TurnBaseToResult()
        result.success = self.result
        userdata.enter_door_result = result
        return 'succeeded'

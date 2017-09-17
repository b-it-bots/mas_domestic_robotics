#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib

from simple_script_server import ScriptAction, ScriptGoal
from mdr_move_tray.msg import MoveTrayFeedback, MoveTrayResult

class SetupMoveTray(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['move_tray_goal'],
                             output_keys=['move_tray_feedback', 'move_tray_result'])

    def execute(self, userdata):
        direction = userdata.move_tray_goal.direction

        feedback = MoveTrayFeedback()
        feedback.current_state = 'MOVE_TRAY'
        feedback.message = '[move_tray] moving the tray ' + direction
        userdata.move_tray_feedback = feedback

        return 'succeeded'

class MoveTray(smach.State):
    def __init__(self, timeout=120.0, action_server='/script_server'):
        smach.State.__init__(self, input_keys=['move_tray_goal'], outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, ScriptAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = ScriptGoal()
        goal.function_name = 'move'
        goal.component_name = 'tray'
        print('GOAL: ' + userdata.move_tray_goal.direction)
        goal.parameter_name = userdata.move_tray_goal.direction

        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.error_code == 0:
            return 'succeeded'
        else:
            return 'failed'

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['move_tray_goal'],
                             output_keys=['move_tray_feedback', 'move_tray_result'])
        self.result = result

    def execute(self, userdata):
        result = MoveTrayResult()
        result.success = self.result
        userdata.move_tray_result = result
        return 'succeeded'

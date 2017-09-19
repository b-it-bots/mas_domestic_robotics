#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import mdr_move_base_safe.msg

class move_base_safe(smach.State):
    def __init__(self, destination_location, timeout=120.0, action_server='move_base_safe_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.destination_location = destination_location
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_move_base_safe.msg.MoveBaseSafeAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_move_base_safe.msg.MoveBaseSafeGoal()
        goal.source_location = 'anywhere'
        goal.destination_location = self.destination_location
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

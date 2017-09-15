#!/usr/bin/python

import rospy
import smach
import smach_ros
import std_msgs.msg
import actionlib
import mdr_actions.msg


class move_base_safe(smach.State):
    def __init__(self, destination_location, timeout=120.0, action_server='move_base_safe_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.destination_location = destination_location
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.MoveBaseSafeAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.MoveBaseSafeGoal()
        goal.source_location = 'anywhere'
        goal.destination_location = self.destination_location
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'


class perceive_table(smach.State):
    def __init__(self, timeout=15.0, action_server='perceive_table_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.PerceiveTableAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.PerceiveTableGoal()
        goal.location = 'anywhere'
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

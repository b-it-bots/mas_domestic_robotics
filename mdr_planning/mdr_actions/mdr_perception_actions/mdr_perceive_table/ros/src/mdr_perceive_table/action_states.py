#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import mdr_perceive_table.msg

class perceive_table(smach.State):
    def __init__(self, timeout=15.0, action_server='perceive_table_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_perceive_table.msg.PerceiveTableAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_perceive_table.msg.PerceiveTableGoal()
        goal.location = 'anywhere'
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

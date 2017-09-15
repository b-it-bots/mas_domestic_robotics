#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mdr_actions.msg import PerceiveTableAction, PerceiveTableGoal

if __name__ == '__main__':
    rospy.init_node('perceive_table_client_tester')
    client = actionlib.SimpleActionClient('/mdr_actions/perceive_table_server', PerceiveTableAction)
    client.wait_for_server()
    goal = PerceiveTableGoal()
    goal.location = 'anywhere'
    try:
        timeout = 45.0
        rospy.loginfo('Sending action lib goal to perceive_table_server')
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        print client.get_result()
    except:
        pass

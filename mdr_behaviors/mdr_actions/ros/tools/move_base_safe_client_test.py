#! /usr/bin/env python
import rospy
import roslib
import actionlib

import sys

from mdr_actions.msg import MoveBaseSafeAction, MoveBaseSafeGoal

if __name__ == '__main__':
    rospy.init_node('move_base_safe_client_tester')
    client = actionlib.SimpleActionClient('move_base_safe_server', MoveBaseSafeAction)
    client.wait_for_server()
    goal = MoveBaseSafeGoal()
    goal.arm_safe_position = 'folded'
    if len(sys.argv) == 3:  # 2 arguments was received : ok proceed
        try:
            goal.source_location = str(sys.argv[1])
            goal.destination_location = str(sys.argv[2])
            timeout = 15.0
            rospy.loginfo('Sending action lib goal to move_base_safe_server, source : ' +
                          goal.source_location + ' , destination : ' + goal.destination_location)
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
            print client.get_result()
        except:
            pass
    else:
        rospy.logerr('Arguments were not received in the proper format !')
        rospy.loginfo('usage : move_base_safe SOURCE DESTINATION')

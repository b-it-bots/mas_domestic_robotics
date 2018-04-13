#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
from tf.transformations import quaternion_from_euler
from actionlib import SimpleActionClient
import move_base_msgs.msg as move_base_msgs
from geometry_msgs.msg import Quaternion, PoseStamped
from mdr_turn_base_to_action.msg import TurnBaseToFeedback, TurnBaseToResult
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal


class SetupTurnBaseTo(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['turn_base_to_feedback'])

    def execute(self, userdata):
        feedback = TurnBaseToFeedback()
        feedback.message = '[turn_base_to_goal] sm initialize'
        userdata.turn_base_to_feedback = feedback

        return 'succeeded'


class TurnBaseTo(smach.State):
    def __init__(self, timeout=120.0,
                 rotation_frame = 'base_link',
                 move_base_server='/move_base',
                 movement_duration=15., speed=0.1):
        smach.State.__init__(self, input_keys=['turn_base_to_goal'], outcomes=['succeeded', 'failed'])

        rospy.loginfo("Using move base server: " + move_base_server)
        rospy.loginfo("Rotation Frame set to : " + rotation_frame)
        self.move_base_server = move_base_server
        self.timeout = timeout
        self.rotation_frame = rotation_frame
        self.movement_duration = movement_duration
        self.speed = speed

        self.entered = False

    def execute(self, userdata):

        move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)

        goal = MoveBaseGoal()
        goal.goal_type = MoveBaseGoal.POSE

        goal.pose.header.frame_id = self.rotation_frame
        q = quaternion_from_euler(0, 0, userdata.turn_base_to_goal.desired_yaw)
        goal.pose.pose.orientation.x  = q[0]
        goal.pose.pose.orientation.y  = q[1]
        goal.pose.pose.orientation.z  = q[2]
        goal.pose.pose.orientation.w  = q[3]
        print ("Goal ", goal)
        move_base_client.wait_for_server()
        move_base_client.send_goal(goal)
        success = move_base_client.wait_for_result()

        if success:
            return 'succeeded'
        else:
            return 'failed'

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['turn_base_to_result'])
        self.result = result

    def execute(self, userdata):
        result = TurnBaseToResult()
        result.success = self.result
        userdata.turn_base_to_result = result
        return 'succeeded'

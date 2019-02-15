#!/usr/bin/python
import rospy
import actionlib
from tf.transformations import quaternion_from_euler
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_turn_base_to_action.msg import TurnBaseToFeedback, TurnBaseToResult

class TurnBaseSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 rotation_frame='base_link',
                 move_base_server='/move_base',
                 max_recovery_attempts=1):
        super(TurnBaseSM, self).__init__('TurnBase', [], max_recovery_attempts)
        self.move_base_server = move_base_server
        self.timeout = timeout
        self.rotation_frame = rotation_frame
        self.move_base_client = None

    def init(self):
        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[turn_base_to] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except:
            rospy.logerr('[turn_base_to] %s server does not seem to respond', self.move_base_server)
        return FTSMTransitions.INITIALISED

    def running(self):
        goal = MoveBaseGoal()
        goal.goal_type = MoveBaseGoal.POSE

        goal.pose.header.frame_id = self.rotation_frame
        q = quaternion_from_euler(0, 0, self.goal.desired_yaw)
        goal.pose.pose.orientation.x = q[0]
        goal.pose.pose.orientation.y = q[1]
        goal.pose.pose.orientation.z = q[2]
        goal.pose.pose.orientation.w = q[3]

        rospy.loginfo('[turn_base_to] Rotating the base to %s', str(self.goal.desired_yaw))
        self.move_base_client.send_goal(goal)
        success = self.move_base_client.wait_for_result()

        if success:
            self.result = self.set_result(True)
        self.result = self.set_result(False)
        return FTSMTransitions.DONE

    def set_result(self, success):
        result = TurnBaseToResult()
        result.success = success
        return result

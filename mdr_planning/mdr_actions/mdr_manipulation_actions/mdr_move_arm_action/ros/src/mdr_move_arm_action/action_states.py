#!/usr/bin/python
import numpy as np

import rospy
import moveit_commander

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_arm_action.msg import MoveArmGoal, MoveArmFeedback, MoveArmResult
from mdr_move_arm_action.dmp import DMPExecutor

class MoveArmSM(ActionSMBase):
    def __init__(self, timeout=120.0, arm_name='arm', max_recovery_attempts=1):
        super(MoveArmSM, self).__init__('MoveArm', [], max_recovery_attempts)
        self.timeout = timeout
        self.arm_name = arm_name
        self.arm = None

    def init(self):
        try:
            self.arm = moveit_commander.MoveGroupCommander(self.arm_name)
        except:
            rospy.logerr('[move_arm] %s could not be initialised', self.arm_name)
        return FTSMTransitions.INITIALISED

    def running(self):
        self.arm.clear_pose_targets()
        success = False
        if self.goal.goal_type == MoveArmGoal.NAMED_TARGET:
            self.arm.set_named_target(self.goal.named_target)
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')
            success = self.arm.go(wait=True)
        elif self.goal.goal_type == MoveArmGoal.END_EFFECTOR_POSE:
            pose = self.goal.end_effector_pose
            dmp_name = self.goal.dmp_name
            tau = self.goal.dmp_tau
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')

            # we use a dynamic motion primitive for moving the arm if one is specified;
            # otherwise, we just use moveit for planning a trajectory and moving the arm
            if dmp_name:
                dmp_traj_executor = DMPExecutor(dmp_name, tau)
                goal = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
                dmp_traj_executor.execute(goal)
            else:
                self.arm.set_pose_reference_frame(pose.header.frame_id)
                self.arm.set_pose_target(pose.pose)
                success = self.arm.go(wait=True)
        elif self.goal.goal_type == MoveArmGoal.JOINT_VALUES:
            joint_values = self.goal.joint_values
            self.arm.set_joint_value_target(joint_values)
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')
            success = self.arm.go(wait=True)
        else:
            rospy.logerr('[move_arm] Invalid target specified; ignoring request')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE

        if not success:
            rospy.logerr('[move_arm] Arm motion unsuccessful')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE

        rospy.loginfo('[move_arm] Arm motion successful')
        self.result = self.set_result(True)
        return FTSMTransitions.DONE

    def set_result(self, success):
        result = MoveArmResult()
        result.success = success
        return result

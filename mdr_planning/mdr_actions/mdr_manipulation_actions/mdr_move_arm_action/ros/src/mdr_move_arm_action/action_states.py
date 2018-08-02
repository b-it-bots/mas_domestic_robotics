#!/usr/bin/python
import numpy as np

import rospy
import smach
import moveit_commander

from mdr_move_arm_action.msg import MoveArmGoal, MoveArmFeedback, MoveArmResult
from mdr_move_arm_action.dmp import DMPExecutor

class SetupMoveArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['move_arm_goal'],
                             output_keys=['move_arm_feedback', 'move_arm_result'])

    def execute(self, userdata):
        # consider moving the other components of the robot out of the arm's way, though
        # this could be dangerous if the robot is for example close to some furniture item

        feedback = MoveArmFeedback()
        feedback.current_state = 'MOVE_ARM'
        feedback.message = '[move_arm] moving the arm'
        userdata.move_arm_feedback = feedback

        return 'succeeded'

class MoveArm(smach.State):
    def __init__(self, timeout=120.0, arm_name='arm'):
        smach.State.__init__(self, input_keys=['move_arm_goal'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.arm = moveit_commander.MoveGroupCommander(arm_name)

    def execute(self, userdata):
        self.arm.clear_pose_targets()
        success = False
        if userdata.move_arm_goal.goal_type == MoveArmGoal.NAMED_TARGET:
            self.arm.set_named_target(userdata.move_arm_goal.named_target)
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')
            success = self.arm.go(wait=True)
        elif userdata.move_arm_goal.goal_type == MoveArmGoal.END_EFFECTOR_POSE:
            pose = userdata.move_arm_goal.end_effector_pose

            dmp_name = userdata.move_arm_goal.dmp_name
            tau = userdata.move_arm_goal.dmp_tau
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
        elif userdata.move_arm_goal.goal_type == MoveArmGoal.JOINT_VALUES:
            joint_values = userdata.move_arm_goal.joint_values
            self.arm.set_joint_value_target(joint_values)
            rospy.loginfo('[move_arm] Planning motion and trying to move arm...')
            success = self.arm.go(wait=True)
        else:
            rospy.logerr('[move_arm] Invalid target specified; ignoring request')
            return 'failed'

        if not success:
            rospy.logerr('[move_arm] Arm motion unsuccessful')
            return 'failed'
        rospy.loginfo('[move_arm] Arm motion successful')
        return 'succeeded'

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['move_arm_goal'],
                             output_keys=['move_arm_feedback', 'move_arm_result'])
        self.result = result

    def execute(self, userdata):
        result = MoveArmResult()
        result.success = self.result
        userdata.move_arm_result = result
        return 'succeeded'

#!/usr/bin/python
from importlib import import_module
import numpy as np

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_pickup_action.msg import PickupGoal, PickupFeedback, PickupResult

class PickupSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 pregrasp_config_name='pregrasp',
                 pregrasp_top_config_name='pregrasp_top',
                 intermediate_grasp_offset=-1,
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 base_elbow_offset=-1.,
                 arm_base_offset=-1.,
                 grasping_orientation=list(),
                 grasping_dmp='',
                 dmp_tau=1.,
                 number_of_retries=0,
                 max_recovery_attempts=1):
        super(PickupSM, self).__init__('Pick', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass()

        self.pregrasp_config_name = pregrasp_config_name
        self.pregrasp_top_config_name = pregrasp_top_config_name
        self.intermediate_grasp_offset = intermediate_grasp_offset
        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.move_forward_server = move_forward_server
        self.base_elbow_offset = base_elbow_offset
        self.arm_base_offset = arm_base_offset
        self.grasping_orientation = grasping_orientation
        self.grasping_dmp = grasping_dmp
        self.dmp_tau = dmp_tau
        self.number_of_retries = number_of_retries

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = None
        self.move_base_client = None
        self.move_forward_client = None

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[pickup] %s server does not seem to respond', self.move_arm_server)

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except:
            rospy.logerr('[pickup] %s server does not seem to respond', self.move_base_server)

        try:
            self.move_forward_client = actionlib.SimpleActionClient(self.move_forward_server, MoveForwardAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_forward_server)
            self.move_forward_client.wait_for_server()
        except:
            rospy.logerr('[pickup] %s server does not seem to respond', self.move_forward_server)

        return FTSMTransitions.INITIALISED

    def running(self):
        pose = self.goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        if self.base_elbow_offset > 0:
            self.__align_base_with_pose(pose_base_link)

            # the base is now correctly aligned with the pose, so we set the
            # y position of the goal pose to the elbow offset
            pose_base_link.pose.position.y = self.base_elbow_offset

        if self.grasping_orientation:
            pose_base_link.pose.orientation.x = self.grasping_orientation[0]
            pose_base_link.pose.orientation.y = self.grasping_orientation[1]
            pose_base_link.pose.orientation.z = self.grasping_orientation[2]
            pose_base_link.pose.orientation.w = self.grasping_orientation[3]

        grasp_successful = False
        retry_count = 0
        while (not grasp_successful) and (retry_count <= self.number_of_retries):
            if retry_count > 0:
                rospy.loginfo('[pickup] Retrying grasp')

            rospy.loginfo('[pickup] Opening the gripper...')
            self.gripper.open()

            rospy.loginfo('[pickup] Preparing for grasp verification')
            self.gripper.init_grasp_verification()

            if self.goal.strategy == PickupGoal.SIDEWAYS_GRASP:
                rospy.loginfo('[pickup] Preparing sideways graps')
                pose_base_link = self.__prepare_sideways_grasp(pose_base_link)

                rospy.loginfo('[pickup] Grasping...')
                arm_motion_success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
                if not arm_motion_success:
                    rospy.logerr('[pickup] Arm motion unsuccessful')
                    self.result = self.set_result(False)
                    return FTSMTransitions.DONE

                rospy.loginfo('[pickup] Arm motion successful')
            elif self.goal.strategy == PickupGoal.TOP_GRASP:
                rospy.loginfo('[pickup] Preparing top grasp')
                pose_base_link, x_align_distance = self.__prepare_top_grasp(pose_base_link)
                self.gripper.orient_z(pose_base_link.pose.orientation)

                pose_base_link, _ = self.__prepare_top_grasp(pose_base_link)
                rospy.loginfo('[pickup] Grasping...')
                arm_motion_success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
                if not arm_motion_success:
                    rospy.logerr('[pickup] Arm motion unsuccessful')
                    self.result = self.set_result(False)
                    return FTSMTransitions.DONE

                rospy.loginfo('[pickup] Arm motion successful')
            else:
                rospy.logerr('[pickup] Unknown grasping strategy requested; ignoring request')
                self.result = self.set_result(False)
                return FTSMTransitions.DONE

            rospy.loginfo('[pickup] Closing the gripper')
            self.gripper.close()

            rospy.loginfo('[pickup] Moving the arm back')
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)

            if self.goal.strategy == PickupGoal.TOP_GRASP:
                rospy.loginfo('[pickup] Moving the base back to the original position')
                if abs(x_align_distance) > 0:
                    self.__move_base_along_x(-x_align_distance)

            rospy.loginfo('[pickup] Verifying the grasp...')
            grasp_successful = self.gripper.verify_grasp()
            if grasp_successful:
                rospy.loginfo('[pickup] Successfully grasped object')
            else:
                rospy.loginfo('[pickup] Grasp unsuccessful')
                retry_count += 1

        if grasp_successful:
            self.result = self.set_result(True)
            return FTSMTransitions.DONE

        rospy.loginfo('[pickup] Grasp could not be performed successfully')
        self.result = self.set_result(False)
        return FTSMTransitions.DONE

    def __align_base_with_pose(self, pose_base_link):
        '''Moves the base so that the elbow is aligned with the goal pose.

        Keyword arguments:
        pose_base_link -- a 'geometry_msgs/PoseStamped' message representing
                          the goal pose in the base link frame

        '''
        aligned_base_pose = PoseStamped()
        aligned_base_pose.header.frame_id = 'base_link'
        aligned_base_pose.header.stamp = rospy.Time.now()
        aligned_base_pose.pose.position.x = 0.
        aligned_base_pose.pose.position.y = pose_base_link.pose.position.y - self.base_elbow_offset
        aligned_base_pose.pose.position.z = 0.
        aligned_base_pose.pose.orientation.x = 0.
        aligned_base_pose.pose.orientation.y = 0.
        aligned_base_pose.pose.orientation.z = 0.
        aligned_base_pose.pose.orientation.w = 1.

        move_base_goal = MoveBaseGoal()
        move_base_goal.goal_type = MoveBaseGoal.POSE
        move_base_goal.pose = aligned_base_pose
        self.move_base_client.send_goal(move_base_goal)
        self.move_base_client.wait_for_result()
        self.move_base_client.get_result()

    def __move_arm(self, goal_type, goal):
        '''Sends a request to the 'move_arm' action server and waits for the
        results of the action execution.

        Keyword arguments:
        goal_type -- 'MoveArmGoal.NAMED_TARGET' or 'MoveArmGoal.END_EFFECTOR_POSE'
        goal -- A string if 'goal_type' is 'MoveArmGoal.NAMED_TARGET';
                a 'geometry_msgs/PoseStamped' if 'goal_type' is 'MoveArmGoal.END_EFFECTOR_POSE'

        '''
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = goal_type
        if goal_type == MoveArmGoal.NAMED_TARGET:
            move_arm_goal.named_target = goal
        elif goal_type == MoveArmGoal.END_EFFECTOR_POSE:
            move_arm_goal.end_effector_pose = goal
            move_arm_goal.dmp_name = self.grasping_dmp
            move_arm_goal.dmp_tau = self.dmp_tau
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        result = self.move_arm_client.get_result()
        return result

    def __prepare_sideways_grasp(self, pose_base_link):
        rospy.loginfo('[PICKUP] Moving to a pregrasp configuration...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)

        if self.intermediate_grasp_offset > 0:
            rospy.loginfo('[PICKUP] Moving to intermediate grasping pose...')
            pose_base_link.pose.position.x -= self.intermediate_grasp_offset
            self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)

        if self.intermediate_grasp_offset > 0:
            pose_base_link.pose.position.x += self.intermediate_grasp_offset
        return pose_base_link

    def __prepare_top_grasp(self, pose_base_link):
        rospy.loginfo('[PICKUP] Moving to a pregrasp configuration...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_top_config_name)
        x_align_distance = 0
        if self.arm_base_offset > 0:
            x_align_distance = pose_base_link.pose.position.x - self.arm_base_offset
            self.__move_base_along_x(x_align_distance)
            pose_base_link.pose.position.x = self.arm_base_offset
        return pose_base_link, x_align_distance

    def __move_base_along_x(self, distance_to_move):
        movement_speed = np.sign(distance_to_move) * 0.1 # m/s
        movement_duration = distance_to_move / movement_speed
        move_forward_goal = MoveForwardGoal()
        move_forward_goal.movement_duration = movement_duration
        move_forward_goal.speed = movement_speed
        self.move_forward_client.send_goal(move_forward_goal)
        self.move_forward_client.wait_for_result()
        self.move_forward_client.get_result()

    def set_result(self, success):
        result = PickupResult()
        result.success = success
        return result

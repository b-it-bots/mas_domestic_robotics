#!/usr/bin/python
from importlib import import_module
import numpy as np

import rospy
import tf
import actionlib
from geometry_msgs.msg import Vector3, Twist

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_push_pull_object_action.msg import PushPullObjectGoal, PushPullObjectResult

class PushPullSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 cmd_vel_topic='/cmd_vel',
                 movement_speed_ms=0.1,
                 number_of_retries=0,
                 max_recovery_attempts=1):
        super(PushPullSM, self).__init__('PushPull', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass()

        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.cmd_vel_topic = cmd_vel_topic
        self.movement_speed_ms = movement_speed_ms
        self.number_of_retries = number_of_retries
        self.max_recovery_attempts = max_recovery_attempts

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = None
        self.cmd_vel_pub = None

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[push_pull] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[push_pull] %s server does not seem to respond %s',
                         self.move_arm_server, str(exc))

        rospy.loginfo('[push_pull] Creating a %s publisher', self.cmd_vel_topic)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        return FTSMTransitions.INITIALISED

    def running(self):
        retry_count = 0
        succeeded = False

        while not succeeded and (retry_count <= self.number_of_retries):
            if retry_count > 0:
                rospy.loginfo('[push_pull] Retrying push')

            object_pose = self.goal.object_pose
            object_pose.header.stamp = rospy.Time(0)
            object_pose_base_link = self.tf_listener.transformPose('base_link', object_pose)

            goal_pose = self.goal.goal_pose
            goal_pose.header.stamp = rospy.Time(0)
            goal_pose_base_link = self.tf_listener.transformPose('base_link', goal_pose)

            pose_diff_vector = Vector3()
            pose_diff_vector.x = goal_pose_base_link.pose.position.x - object_pose_base_link.pose.position.x
            pose_diff_vector.y = goal_pose_base_link.pose.position.y - object_pose_base_link.pose.position.y

            rospy.loginfo('[push_pull] Pushing the object')
            self.__push_pull_object(pose_diff_vector,
                                    self.goal.goal_distance_tolerance_m,
                                    self.goal.context)

            # TODO: reintegrate fault detection!
            succeeded = True

        if succeeded:
            self.result = self.set_result(True)
            return FTSMTransitions.DONE

        self.result = self.set_result(False)
        return FTSMTransitions.DONE

    def __move_arm(self, goal_type, goal):
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

    def __push_pull_object(self, distance_to_move, goal_tolerance, context):
        motion_duration_x = abs(distance_to_move.x) / self.movement_speed_ms
        motion_duration_y = abs(distance_to_move.y) / self.movement_speed_ms

        # we push the object by moving the base
        rospy.loginfo('[push_pull] Pushing...')
        self.__send_base_vel(np.sign(distance_to_move.x) * self.movement_speed_ms,
                             np.sign(distance_to_move.y) * self.movement_speed_ms,
                             motion_duration_x,
                             motion_duration_y)

        if context != PushPullObjectGoal.CONTEXT_TABLETOP_MANIPULATION:
            rospy.loginfo('[push_pull] Releasing the object')
            self.gripper.open()
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)

            # we return the base back to its original position
            rospy.loginfo('[push_pull] Moving back...')
            self.__send_base_vel(-np.sign(distance_to_move.x) * self.movement_speed_ms,
                                 -np.sign(distance_to_move.y) * self.movement_speed_ms,
                                 motion_duration_x,
                                 motion_duration_y)

    def __send_base_vel(self, vel_x, vel_y, motion_duration_x, motion_duration_y):
        duration_x = rospy.Duration.from_sec(motion_duration_x)
        duration_y = rospy.Duration.from_sec(motion_duration_y)
        max_duration = rospy.Duration.from_sec(max(motion_duration_x,
                                                   motion_duration_y))

        rate = rospy.Rate(5)
        twist = Twist()
        twist.linear.x = vel_x
        twist.linear.y = vel_y

        start_time = rospy.Time.now()
        time_diff = rospy.Time.now() - start_time
        while time_diff < max_duration:
            if time_diff >= duration_x:
                twist.linear.x = 0.
            if time_diff >= duration_y:
                twist.linear.y = 0.
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
            time_diff = rospy.Time.now() - start_time

        # we publish a zero twist at the end so that the robot stops moving
        zero_twist = Twist()
        self.cmd_vel_pub.publish(zero_twist)

    def set_result(self, success):
        result = PushPullObjectResult()
        result.success = success
        return result

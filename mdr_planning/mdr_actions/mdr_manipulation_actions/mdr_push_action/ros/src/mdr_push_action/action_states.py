#!/usr/bin/python
from importlib import import_module
import numpy as np

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Vector3, Twist

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_push_action.msg import PushGoal, PushFeedback, PushResult

class PushSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 pregrasp_config_name='pregrasp',
                 pregrasp_low_config_name='pregrasp_low',
                 pregrasp_height_threshold=0.5,
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 cmd_vel_topic='/cmd_vel',
                 movement_speed_ms=0.1,
                 base_elbow_offset=-1.,
                 grasping_orientation=list(),
                 grasping_dmp='',
                 dmp_tau=1.,
                 number_of_retries=0,
                 max_recovery_attempts=1):
        super(PushSM, self).__init__('Push', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass()

        self.pregrasp_config_name = pregrasp_config_name
        self.pregrasp_low_config_name = pregrasp_low_config_name
        self.pregrasp_height_threshold = pregrasp_height_threshold
        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.cmd_vel_topic = cmd_vel_topic
        self.movement_speed_ms = movement_speed_ms
        self.base_elbow_offset = base_elbow_offset
        self.grasping_orientation = grasping_orientation
        self.grasping_dmp = grasping_dmp
        self.dmp_tau = dmp_tau
        self.number_of_retries = number_of_retries
        self.max_recovery_attempts = max_recovery_attempts

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = None
        self.move_base_client = None
        self.cmd_vel_pub = None

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[push] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[push] %s server does not seem to respond', self.move_arm_server)

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[push] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except:
            rospy.logerr('[push] %s server does not seem to respond', self.move_base_server)

        rospy.loginfo('[push] Creating a %s publisher', self.cmd_vel_topic)
        self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)

        return FTSMTransitions.INITIALISED

    def running(self):
        '''
        Fault detection algorithm checks two things: pushing and grasping. If at least one of them raise the
        fault alarm, the whole pushing action are repeated.

        Pushing action flow:
        1. Initial Movement
            The robot starts the action by opening the gripper and moving the manipulator to neutral position.
        2. Reaching the Object
            The end effector pose are fixed beside the z-axis pose. The z-axis pose will be set using normal
            distribution for certain range. Therefore the robot will not grasp the object at the same location
            all the time.
        3. Grasp the Object
            An image is taken after the grasping is done. The success result is determined by comparing the image
            with the model. (1st fault detection)
        4. Push the Object
            The robot moves its base forward while maintaining the manipulator configuration to emulates
            the push action. Force data starting from before the base moves forward until it stops are saved in
            the list. The mean value is calculated and compared with the set threshold. (2nd fault detection)
        5. Release the Object
        6. Return to Initial Position
            The robot returns to initial position by first retracting its manipulator to neutral position,
            then move its base back. If both fault detection returns false (success == True), it will exit the
            loop function. The variable retry_count determine how many retries the algorithm allows until the
            pushing action is done properly.
        '''

        retry_count = 0
        succeeded = False

        while not succeeded and (retry_count <= self.number_of_retries):
            if retry_count > 0:
                rospy.loginfo('[push] Retrying push')

            object_pose = self.goal.object_pose
            object_pose.header.stamp = rospy.Time(0)
            object_pose_base_link = self.tf_listener.transformPose('base_link', object_pose)

            goal_pose = self.goal.goal_pose
            goal_pose.header.stamp = rospy.Time(0)
            goal_pose_base_link = self.tf_listener.transformPose('base_link', goal_pose)

            if self.base_elbow_offset > 0:
                self.__align_base_with_pose(object_pose_base_link)

                # the base is now correctly aligned with the pose, so we set the
                # y position of the goal pose to the elbow offset
                object_pose_base_link.pose.position.y = self.base_elbow_offset

            if self.grasping_orientation:
                object_pose_base_link.pose.orientation.x = self.grasping_orientation[0]
                object_pose_base_link.pose.orientation.y = self.grasping_orientation[1]
                object_pose_base_link.pose.orientation.z = self.grasping_orientation[2]
                object_pose_base_link.pose.orientation.w = self.grasping_orientation[3]

            rospy.loginfo('[push] Opening gripper')
            self.gripper.open()

            rospy.loginfo('[push] Preparing sideways graps')
            object_pose_base_link = self.__prepare_sideways_grasp(object_pose_base_link)

            rospy.loginfo('[push] Reaching the object...')
            arm_motion_success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE,
                                                 object_pose_base_link)
            if not arm_motion_success:
                rospy.logerr('[push] Arm motion unsuccessful')
                self.result = self.set_result(False)
                return FTSMTransitions.DONE

            rospy.loginfo('[push] Grasping the object')
            self.gripper.close()

            pose_diff_vector = Vector3()
            pose_diff_vector.x = goal_pose_base_link.pose.position.x - object_pose_base_link.pose.position.x
            pose_diff_vector.y = goal_pose_base_link.pose.position.y - object_pose_base_link.pose.position.y

            rospy.loginfo('[push] Pushing the object')
            self.__push_object(pose_diff_vector, self.goal.goal_distance_tolerance_m)

            rospy.loginfo('[push] Releasing the object')
            self.gripper.open()
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)

            # TODO: reintegrate fault detection!
            succeeded = True

        if succeeded:
            self.result = self.set_result(True)
            return FTSMTransitions.DONE

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

    def __prepare_sideways_grasp(self, pose_base_link):
        rospy.loginfo('[push] Moving to a pregrasp configuration...')
        if pose_base_link.pose.position.z > self.pregrasp_height_threshold:
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)
        else:
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_low_config_name)
        return pose_base_link

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

    def __push_object(self, distance_to_move, goal_tolerance):
        motion_duration_x = distance_to_move.x / self.movement_speed_ms
        motion_duration_y = distance_to_move.y / self.movement_speed_ms

        # we push the object by moving the base
        self.__send_base_vel(np.sign(distance_to_move.x) * self.movement_speed_ms,
                             np.sign(distance_to_move.y) * self.movement_speed_ms,
                             motion_duration_x,
                             motion_duration_y)

        # we return the base back to its original position
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
        result = PushResult()
        result.success = success
        return result

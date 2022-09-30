#!/usr/bin/python
from importlib import import_module
import numpy as np
from std_srvs.srv import Empty

from std_msgs.msg import String

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_pickup_action.msg import PickupGoal, PickupResult

import pdb

class PickupSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 pregrasp_config_name='pregrasp',
                 pregrasp_top_config_name='pregrasp_top',
                 pregrasp_low_config_name='pregrasp_low',
                 pregrasp_height_threshold=0.5,
                 intermediate_grasp_offset=-1,
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 base_elbow_offset=-1.,
                 arm_base_offset=-1.,
                 grasping_orientation=None,
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
        self.pregrasp_low_config_name = pregrasp_low_config_name
        self.pregrasp_height_threshold = pregrasp_height_threshold
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

        self.target_pose_pub = rospy.Publisher('target_pose', PoseStamped, queue_size=1)
        self.joint_states_pub = rospy.Subscriber('/hsrb/joint_states', JointState, self.joint_states_cb)

        self.move_arm_client = None
        self.move_base_client = None
        self.move_forward_client = None

        self.say_pub = rospy.Publisher('/say', String, latch=True, queue_size=1)

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                         self.move_arm_server, str(exc))

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                         self.move_base_server, str(exc))

        try:
            self.move_forward_client = actionlib.SimpleActionClient(self.move_forward_server, MoveForwardAction)
            rospy.loginfo('[pickup] Waiting for %s server', self.move_forward_server)
            self.move_forward_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] %s server does not seem to respond: %s',
                         self.move_forward_server, str(exc))

        rospy.loginfo('Waiting for clear_octomap service')
        rospy.wait_for_service('/clear_octomap')
        rospy.loginfo('Found /clear_octomap service')
        self.clear_octomap_service = rospy.ServiceProxy('/clear_octomap', Empty)

        return FTSMTransitions.INITIALISED

    def running(self):
        pose = self.goal.pose
        self.target_pose_pub.publish(self.goal.pose)
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        pose_base_link.pose.position.x -= 0.01

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
                self.say('Preparing sideways grasp')
                rospy.loginfo('[pickup] Preparing sideways grasp')
                pose_base_link = self.__prepare_sideways_grasp(pose_base_link)

                rospy.loginfo('[pickup] Grasping...')
                arm_motion_success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
                if not arm_motion_success:
                    rospy.logerr('[pickup] Arm motion unsuccessful')
                    self.result = self.set_result(False)
                    return FTSMTransitions.DONE

                rospy.loginfo('[pickup] Arm motion successful')
            elif self.goal.strategy == PickupGoal.TOP_GRASP:
                self.say('Preparing top grasp')
                rospy.loginfo('[pickup] Preparing top grasp')
                pose_base_link, x_align_distance = self.__prepare_top_grasp(pose_base_link)
                self.gripper.orient_z(pose_base_link.pose.orientation)

                # pose_base_link, _ = self.__prepare_top_grasp(pose_base_link)
                rospy.loginfo('[pickup] Grasping...')
                arm_motion_success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
                if not arm_motion_success:
                    rospy.logerr('[pickup] Arm motion unsuccessful')
                    self.result = self.set_result(False)
                    return FTSMTransitions.DONE

                rospy.loginfo('[pickup] Arm motion successful')
            else:
                self.say('Unknown grasp strategy')
                rospy.logerr('[pickup] Unknown grasping strategy requested; ignoring request')
                self.result = self.set_result(False)
                return FTSMTransitions.DONE

            rospy.loginfo('[pickup] Closing the gripper')
            self.gripper.close()
            rospy.loginfo('[pickup] Clearing octomap')
            self.clear_octomap_service()

            if self.goal.context != PickupGoal.CONTEXT_TABLETOP_MANIPULATION:
                rospy.loginfo('[pickup] Moving the arm back')
                current_joint_pos = self.joint_states
                arm_joint_pos_indices = [current_joint_pos.name.index('arm_lift_joint'), current_joint_pos.name.index('arm_flex_joint'),
                                         current_joint_pos.name.index('arm_roll_joint'), current_joint_pos.name.index('wrist_flex_joint'),
                                         current_joint_pos.name.index('wrist_roll_joint')]
                arm_joint_pos = [self.joint_states.position[idx] for idx in arm_joint_pos_indices]
                arm_joint_pos.append(0.0)
                # Added an offset to the arm_lift_joint to avoid collision with the table
                arm_joint_pos[0] += 0.08
                self.__move_arm(MoveArmGoal.JOINT_VALUES, arm_joint_pos)
                rospy.loginfo('\n[pickup] ***************************************\n')
                rospy.logwarn('[pickup] Arm moving up after pickup')
                self.__move_base_along_x(-0.1)
                rospy.logwarn('[pickup] Base moving back after pickup')
                self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)
                rospy.logwarn('[pickup] Arm moving to pregrasp after pickup')
                self.__move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)
                rospy.logwarn('[pickup] Arm moving to safe config after pickup')

                if self.goal.strategy == PickupGoal.TOP_GRASP:
                    rospy.loginfo('[pickup] Moving the base back to the original position')
                    if abs(x_align_distance) > 0:
                        self.__move_base_along_x(-x_align_distance)

            rospy.loginfo('[pickup] Verifying the grasp...')
            rospy.loginfo('\n[pickup] END***************************************\n')
            # Added a condition to check if the object is grasped or not by using hand_motor_joint 'closed' value as the min threshold
            grasp_successful = (self.joint_states.position[self.joint_states.name.index('hand_motor_joint')]) > -0.83 # NOTE: -0.83 value comes from thinnest object which is toothbrush

            if grasp_successful:
                rospy.loginfo('[pickup] Successfully grasped object')
                grasp_successful=True

            # check grasping based on force-torque value             
            elif self.gripper.verify_grasp():
                rospy.loginfo('[pickup] Grasp F-T verification failed, but gripper has grasped something')
                grasp_successful=True
            else:
                rospy.loginfo('[pickup] Grasp F-T verification failed along with "hand_motor_joint" value threshold, but the gripper is still open; retrying')
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

    def __move_arm(self, goal_type, goal, dmp_flag=True):
        '''Sends a request to the 'move_arm' action server and waits for the
        results of the action execution.

        Keyword arguments:
        goal_type -- 'MoveArmGoal.NAMED_TARGET' or 'MoveArmGoal.END_EFFECTOR_POSE'
        goal -- A string if 'goal_type' is 'MoveArmGoal.NAMED_TARGET';
                a 'geometry_msgs/PoseStamped' if 'goal_type' is 'MoveArmGoal.END_EFFECTOR_POSE'

        '''
        rospy.loginfo('[pickup] I am inside _move_arm')
        # pdb.set_trace()
        move_arm_goal = MoveArmGoal()
        move_arm_goal.goal_type = goal_type
        if goal_type == MoveArmGoal.NAMED_TARGET:
            move_arm_goal.named_target = goal
        elif goal_type == MoveArmGoal.END_EFFECTOR_POSE:
            move_arm_goal.end_effector_pose = goal
            if dmp_flag:
                move_arm_goal.dmp_name = self.grasping_dmp
                move_arm_goal.dmp_tau = self.dmp_tau
        elif goal_type == MoveArmGoal.JOINT_VALUES:
            move_arm_goal.joint_values = goal
                
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        result = self.move_arm_client.get_result()
        return result

    def __prepare_sideways_grasp(self, pose_base_link):
        rospy.loginfo('[PICKUP] Moving to a pregrasp configuration...')
        if pose_base_link.pose.position.z > self.pregrasp_height_threshold:
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)
        else:
            self.__move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_low_config_name)

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

    def joint_states_cb(self, msg):
        self.joint_states = msg
    
    def say(self, sentence):
        say_msg = String()
        say_msg.data = sentence
        self.say_pub.publish(say_msg)


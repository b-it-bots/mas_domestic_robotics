#!/usr/bin/python
from importlib import import_module

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_place_action.msg import PlaceGoal, PlaceFeedback, PlaceResult

class PlaceSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 preplace_config_name='pregrasp',
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 base_elbow_offset=-1.,
                 placing_orientation=list(),
                 placing_dmp='',
                 dmp_tau=1.,
                 max_recovery_attempts=1):
        super(PlaceSM, self).__init__('Place', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass()

        self.preplace_config_name = preplace_config_name
        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.base_elbow_offset = base_elbow_offset
        self.placing_orientation = placing_orientation
        self.placing_dmp = placing_dmp
        self.dmp_tau = dmp_tau

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = None
        self.move_base_client = None

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[place] Waiting for % server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[place] %s server does not seem to respond', self.move_arm_server)

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[place] Waiting for % server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except:
            rospy.logerr('[place] %s server does not seem to respond', self.move_base_server)

        return FTSMTransitions.INITIALISED

    def running(self):
        pose = self.goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)
        if self.placing_orientation:
            pose_base_link.pose.orientation.x = self.placing_orientation[0]
            pose_base_link.pose.orientation.y = self.placing_orientation[1]
            pose_base_link.pose.orientation.z = self.placing_orientation[2]
            pose_base_link.pose.orientation.w = self.placing_orientation[3]

        if self.base_elbow_offset > 0:
            self.__align_base_with_pose(pose_base_link)

            # the base is now correctly aligned with the pose, so we set the
            # y position of the goal pose to the elbow offset
            pose_base_link.pose.position.y = self.base_elbow_offset

        rospy.loginfo('[place] Moving to a preplace configuration...')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.preplace_config_name)

        # we set up the arm group for moving
        rospy.loginfo('[place] Placing...')
        success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
        if not success:
            rospy.logerr('[place] Arm motion unsuccessful')
            self.result = self.set_result(False)
            return FTSMTransitions.DONE

        rospy.loginfo('[place] Arm motion successful')
        rospy.loginfo('[place] Opening the gripper...')
        self.gripper.open()

        rospy.loginfo('[place] Moving the arm back')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)
        self.result = self.set_result(True)
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
            move_arm_goal.dmp_name = self.placing_dmp
            move_arm_goal.dmp_tau = self.dmp_tau
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        result = self.move_arm_client.get_result()
        return result

    def set_result(self, success):
        result = PlaceResult()
        result.success = success
        return result

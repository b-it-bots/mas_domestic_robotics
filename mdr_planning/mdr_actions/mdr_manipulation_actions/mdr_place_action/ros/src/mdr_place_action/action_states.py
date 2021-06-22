#!/usr/bin/python
from importlib import import_module
import numpy as np

import rospy
import tf
import actionlib
from geometry_msgs.msg import PoseStamped, Twist

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_place_action.msg import PlaceResult

from mas_hsr_move_arm_joints_action.msg import MoveArmJointsAction, MoveArmJointsGoal

class PlaceSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gripper_controller_pkg_name='mdr_gripper_controller',
                 preplace_config_name='pregrasp',
                 preplace_low_config_name='neutral',
                 preplace_height_threshold=0.6,
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 base_elbow_offset=-1.,
                 placing_orientation=None,
                 placing_dmp='',
                 dmp_tau=1.,
                 downward_placing_vel=-0.02,
                 max_recovery_attempts=1):
        super(PlaceSM, self).__init__('Place', [], max_recovery_attempts)
        self.timeout = timeout

        gripper_controller_module_name = '{0}.gripper_controller'.format(gripper_controller_pkg_name)
        GripperControllerClass = getattr(import_module(gripper_controller_module_name),
                                         'GripperController')
        self.gripper = GripperControllerClass(joint_vel=downward_placing_vel)

        self.preplace_config_name = preplace_config_name
        self.preplace_low_config_name = preplace_low_config_name
        self.preplace_height_threshold = preplace_height_threshold
        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.move_forward_server = move_forward_server
        self.base_elbow_offset = base_elbow_offset
        self.placing_orientation = placing_orientation
        self.placing_dmp = placing_dmp
        self.dmp_tau = dmp_tau

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = None
        self.move_base_client = None
        self.move_forward_client = None
        self.move_arm_joints_client = None
        self.base_vel_pub = None

    def init(self):
        try:
            self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
            rospy.loginfo('[place] Waiting for %s server', self.move_arm_server)
            self.move_arm_client.wait_for_server()
        except:
            rospy.logerr('[place] %s server does not seem to respond', self.move_arm_server)

        try:
            self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
            rospy.loginfo('[place] Waiting for %s server', self.move_base_server)
            self.move_base_client.wait_for_server()
        except:
            rospy.logerr('[place] %s server does not seem to respond', self.move_base_server)

        try:
            self.move_forward_client = actionlib.SimpleActionClient(self.move_forward_server, MoveForwardAction)
            rospy.loginfo('[place] Waiting for %s server', self.move_forward_server)
            self.move_forward_client.wait_for_server()
        except:
            rospy.logerr('[place] %s server does not seem to respond', self.move_forward_server)

        try:
            self.move_arm_joints_client = actionlib.SimpleActionClient('mas_hsr_move_arm_joints_server', MoveArmJointsAction)
            rospy.loginfo('[pickup] Waiting for server mas_hsr_move_arm_joints_server')
            self.move_arm_joints_client.wait_for_server()
        except Exception as exc:
            rospy.logerr('[pickup] mas_hsr_move_arm_joints_server server does not seem to respond: %s', str(exc))

        self.base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)

        return FTSMTransitions.INITIALISED

    def running(self):
        pose = self.goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        _, base_link_map_rot = self.get_transform('map', 'base_link', rospy.Time.now())
        euler_rotation = tf.transformations.euler_from_quaternion(base_link_map_rot)
        orientation_before_alignment = euler_rotation[2]

        if self.placing_orientation is not None:
            pose_base_link.pose.orientation.x = self.placing_orientation[0]
            pose_base_link.pose.orientation.y = self.placing_orientation[1]
            pose_base_link.pose.orientation.z = self.placing_orientation[2]
            pose_base_link.pose.orientation.w = self.placing_orientation[3]

        if self.base_elbow_offset > 0:
            self.__align_base_with_pose(pose_base_link)

            # the base is now correctly aligned with the pose, so we set the
            # y position of the goal pose to the elbow offset
            pose_base_link.pose.position.y = self.base_elbow_offset

        # rospy.loginfo('[place] Moving to a preplace configuration...')
        # if pose_base_link.pose.position.z > self.preplace_height_threshold:
        #     self.__move_arm(MoveArmGoal.NAMED_TARGET, self.preplace_config_name)
        # else:
        #     self.__move_arm(MoveArmGoal.NAMED_TARGET, self.preplace_low_config_name)
        #
        # # we set up the arm group for moving
        # rospy.loginfo('[place] Placing...')
        # success = self.__move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
        # if not success:
        #     rospy.logerr('[place] Arm motion unsuccessful')
        #     self.result = self.set_result(False)
        #     return FTSMTransitions.DONE
        # rospy.loginfo('[place] Arm motion successful')

        self.align_base_with_orientation(orientation_before_alignment)

        goal = MoveArmJointsGoal()
        goal.arm_joint_names = ['arm_flex_joint', 'arm_roll_joint',
                                'arm_lift_joint', 'wrist_roll_joint', 'wrist_flex_joint']
        goal.arm_joint_values = [-1.57, 0., min(0.69, pose_base_link.pose.position.z), 0., -1.57]
        self.move_arm_joints_client.send_goal(goal)
        self.move_arm_joints_client.wait_for_result()

        self.__move_base_along_x(pose_base_link.pose.position.x-0.55)
        # self.align_base_with_orientation(orientation_before_alignment)

        # the arm is moved down until it makes an impact with the placing surface
        if self.goal.release_on_impact:
            rospy.loginfo('[place] Moving arm down until surface impact is detected...')
            self.gripper.init_impact_detection_z()
            while not self.gripper.detect_impact_z():
                self.gripper.move_down()
                rospy.sleep(0.1)
            self.gripper.stop_arm()
            rospy.loginfo('[place] Impact detected')

        # the object can be released now that impact with the surface has been made
        rospy.loginfo('[place] Opening the gripper...')
        self.gripper.open()

        rospy.loginfo('[place] Moving the arm back')
        self.__move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)
        self.result = self.set_result(True)

        self.__move_base_along_x(-0.2)
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

    def __move_base_along_x(self, distance_to_move):
        aligned_base_pose = PoseStamped()
        aligned_base_pose.header.frame_id = 'base_link'
        aligned_base_pose.header.stamp = rospy.Time.now()
        aligned_base_pose.pose.position.x = distance_to_move
        aligned_base_pose.pose.orientation.w = 1.

        self.tf_listener.waitForTransform('map', 'base_link', rospy.Time.now(), rospy.Duration(5))
        goal_pose_map = self.tf_listener.transformPose('map', aligned_base_pose)

        movement_speed_linear = 0.05
        movement_speed_angular = 0.01

        twist_msg = Twist()
        goal_reached = False
        while not goal_reached:
            goal_pose_map.header.stamp = rospy.Time.now()
            self.tf_listener.waitForTransform('base_link', 'map', rospy.Time.now(), rospy.Duration(5))
            goal_base_link = self.tf_listener.transformPose('base_link', goal_pose_map)
            euler_rotation = tf.transformations.euler_from_quaternion([goal_base_link.pose.orientation.x,
                                                                       goal_base_link.pose.orientation.y,
                                                                       goal_base_link.pose.orientation.z,
                                                                       goal_base_link.pose.orientation.w])
            goal_rotation = euler_rotation[2]
            if abs(goal_base_link.pose.position.x) > 0:
                twist_msg.linear.x = np.sign(goal_base_link.pose.position.x) * movement_speed_linear
            if abs(goal_base_link.pose.position.y) > 0:
                twist_msg.linear.y = np.sign(goal_base_link.pose.position.y) * movement_speed_linear
            if abs(goal_rotation) > 0:
                twist_msg.angular.z = np.sign(goal_rotation) * movement_speed_angular
            # rospy.logwarn('%s %s %s',
            #               goal_base_link.pose.position.x,
            #               goal_base_link.pose.position.y,
            #               goal_rotation)

            goal_reached = abs(goal_base_link.pose.position.x) < 0.05 and\
                           abs(goal_base_link.pose.position.y) < 1e-2 and\
                           abs(goal_rotation) < 1e-2
            self.base_vel_pub.publish(twist_msg)
            rospy.sleep(0.01)

        self.base_vel_pub.publish(Twist())

        # movement_speed = np.sign(distance_to_move) * 0.1 # m/s
        # movement_duration = distance_to_move / movement_speed
        # move_forward_goal = MoveForwardGoal()
        # move_forward_goal.movement_duration = movement_duration
        # move_forward_goal.speed = movement_speed
        # self.move_forward_client.send_goal(move_forward_goal)
        # self.move_forward_client.wait_for_result()
        # self.move_forward_client.get_result()

    def set_result(self, success):
        result = PlaceResult()
        result.success = success
        return result

    def get_transform(self, target_frame, source_frame, tf_time):
        '''Returns the translation and rotation of the source frame
        with respect to the target frame at the given time.

        Keyword arguments:
        target_frame: str -- name of the transformation target frame
        source_frame: str -- name of the transformation source frame
        tf_time: rospy.rostime.Time -- time of the transform

        '''
        trans = None
        rot = None
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(target_frame, source_frame, tf_time)
                break
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
        return (trans, rot)

    def align_base_with_orientation(self, orientation):
        _, base_link_map_rot = self.get_transform('map', 'base_link', rospy.Time.now())
        euler_rotation = tf.transformations.euler_from_quaternion(base_link_map_rot)
        current_rotation = euler_rotation[2]
        aligned = abs(orientation - current_rotation) < 1e-2

        alignment_direction = np.sign(orientation - current_rotation)
        twist_msg = Twist()
        twist_msg.angular.z = alignment_direction * 0.05
        while not aligned:
            self.base_vel_pub.publish(twist_msg)
            _, base_link_map_rot = self.get_transform('map', 'base_link', rospy.Time.now())
            euler_rotation = tf.transformations.euler_from_quaternion(base_link_map_rot)
            current_rotation = euler_rotation[2]
            aligned = abs(orientation - current_rotation) < 1e-2

#!/usr/bin/python

import rospy
import smach
import tf
import actionlib
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_pickup_action.msg import PickupGoal, PickupFeedback, PickupResult

class SetupPickup(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['pickup_goal'],
                             output_keys=['pickup_feedback', 'pickup_result'])

    def execute(self, userdata):
        # consider moving the other components of the robot out of the arm's way, though
        # this could be dangerous if the robot is for example close to some furniture item

        feedback = PickupFeedback()
        feedback.current_state = 'SETUP_PICKUP'
        feedback.message = '[SETUP_PICKUP] setting up the arm'
        userdata.pickup_feedback = feedback

        return 'succeeded'

class Pickup(smach.State):
    def __init__(self, timeout=120.0,
                 gripper_joint_names=list(),
                 open_gripper_joint_values=list(),
                 gripper_cmd_topic='/gripper/command',
                 pregrasp_config_name='pregrasp',
                 intermediate_grasp_offset=-1,
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 base_elbow_offset=-1.,
                 grasping_orientation=list(),
                 grasping_dmp='',
                 dmp_tau=1.):
        smach.State.__init__(self, input_keys=['pickup_goal'],
                             output_keys=['pickup_feedback'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.gripper_joint_names = gripper_joint_names
        self.open_gripper_joint_values = open_gripper_joint_values
        self.gripper_traj_pub = rospy.Publisher(gripper_cmd_topic,
                                                JointTrajectory,
                                                queue_size=10)
        self.pregrasp_config_name = pregrasp_config_name
        self.intermediate_grasp_offset = intermediate_grasp_offset
        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.move_base_server = move_base_server
        self.base_elbow_offset = base_elbow_offset
        self.grasping_orientation = grasping_orientation
        self.grasping_dmp = grasping_dmp
        self.dmp_tau = dmp_tau

        self.tf_listener = tf.TransformListener()

        self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
        self.move_arm_client.wait_for_server()

        self.move_base_client = actionlib.SimpleActionClient(self.move_base_server, MoveBaseAction)
        self.move_base_client.wait_for_server()

    def execute(self, userdata):
        feedback = PickupFeedback()
        feedback.current_state = 'PICKUP'
        feedback.message = '[PICKUP] moving the arm'
        userdata.pickup_feedback = feedback

        pose = userdata.pickup_goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        if self.base_elbow_offset > 0:
            self.align_base_with_pose(pose_base_link)

            # the base is now correctly aligned with the pose, so we set the
            # y position of the goal pose to the elbow offset
            pose_base_link.pose.position.y = self.base_elbow_offset

        if self.grasping_orientation:
            pose_base_link.pose.orientation.x = self.grasping_orientation[0]
            pose_base_link.pose.orientation.y = self.grasping_orientation[1]
            pose_base_link.pose.orientation.z = self.grasping_orientation[2]
            pose_base_link.pose.orientation.w = self.grasping_orientation[3]

        # we open the gripper
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = self.open_gripper_joint_values
        trajectory_point.time_from_start = rospy.Time(5.)
        traj.points = [trajectory_point]
        self.gripper_traj_pub.publish(traj)
        rospy.sleep(3.)

        rospy.loginfo('[PICKUP] Moving to a pregrasp configuration...')
        self.move_arm(MoveArmGoal.NAMED_TARGET, self.pregrasp_config_name)

        if self.intermediate_grasp_offset > 0:
            rospy.loginfo('[PICKUP] Moving to intermediate grasping pose...')
            pose_base_link.pose.position.x -= self.intermediate_grasp_offset
            self.move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)

        rospy.loginfo('[PICKUP] Grasping...')
        if self.intermediate_grasp_offset > 0:
            pose_base_link.pose.position.x += self.intermediate_grasp_offset
        success = self.move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
        if not success:
            rospy.logerr('[pickup] Arm motion unsuccessful')
            return 'failed'

        rospy.loginfo('[PICKUP] Arm motion successful')
        rospy.loginfo('[PICKUP] Closing the gripper')

        # we move the gripper joints to a suitable position
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = userdata.pickup_goal.closed_gripper_joint_values
        trajectory_point.time_from_start = rospy.Time(5.)
        traj.points = [trajectory_point]
        self.gripper_traj_pub.publish(traj)
        rospy.sleep(3.)

        rospy.loginfo('[PICKUP] Moving the arm back')
        self.move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)

        return 'succeeded'

    def align_base_with_pose(self, pose_base_link):
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

    def move_arm(self, goal_type, goal):
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

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['pickup_goal'],
                             output_keys=['pickup_feedback', 'pickup_result'])
        self.result = result

    def execute(self, userdata):
        result = PickupResult()
        result.success = self.result
        userdata.pickup_result = result
        return 'succeeded'

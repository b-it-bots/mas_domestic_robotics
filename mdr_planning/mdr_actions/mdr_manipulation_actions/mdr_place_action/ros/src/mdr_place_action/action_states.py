#!/usr/bin/python

import rospy
import smach
import tf
import actionlib
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mdr_move_arm_action.msg import MoveArmAction, MoveArmGoal
from mdr_place_action.msg import PlaceGoal, PlaceFeedback, PlaceResult

class SetupPlace(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['place_goal'],
                             output_keys=['place_feedback', 'place_result'])

    def execute(self, userdata):
        # consider moving the other components of the robot out of the arm's way, though
        # this could be dangerous if the robot is for example close to some furniture item

        feedback = PlaceFeedback()
        feedback.current_state = 'SETUP_PLACE'
        feedback.message = '[SETUP_PLACE] setting up the arm'
        userdata.place_feedback = feedback

        return 'succeeded'

class Place(smach.State):
    def __init__(self, timeout=120.0,
                 gripper_joint_names=list(),
                 gripper_joint_values=list(),
                 gripper_cmd_topic='/gripper/command',
                 preplace_config_name='pregrasp',
                 safe_arm_joint_config='folded',
                 move_arm_server='move_arm_server'):
        smach.State.__init__(self, input_keys=['place_goal'],
                             output_keys=['place_feedback'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.gripper_joint_names = gripper_joint_names
        self.gripper_joint_values = gripper_joint_values
        self.gripper_traj_pub = rospy.Publisher(gripper_cmd_topic, JointTrajectory,
                                                queue_size=10)
        self.preplace_config_name = preplace_config_name
        self.safe_arm_joint_config = safe_arm_joint_config
        self.move_arm_server = move_arm_server
        self.tf_listener = tf.TransformListener()

        self.move_arm_client = actionlib.SimpleActionClient(self.move_arm_server, MoveArmAction)
        self.move_arm_client.wait_for_server()

    def execute(self, userdata):
        feedback = PlaceFeedback()
        feedback.current_state = 'PLACE'
        feedback.message = '[PLACE] moving the arm'
        userdata.place_feedback = feedback

        self.move_arm(MoveArmGoal.NAMED_TARGET, self.preplace_config_name)

        pose = userdata.place_goal.pose
        pose.header.stamp = rospy.Time(0)
        pose_base_link = self.tf_listener.transformPose('base_link', pose)

        # we set up the arm group for moving
        rospy.loginfo('[PLACE] Placing...')
        success = self.move_arm(MoveArmGoal.END_EFFECTOR_POSE, pose_base_link)
        if not success:
            rospy.logerr('[PLACE] Arm motion unsuccessful')
            return 'failed'

        rospy.loginfo('[PLACE] Arm motion successful')
        rospy.loginfo('[PLACE] Opening the gripper')

        # we move the gripper joints to a suitable position
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joint_names
        trajectory_point = JointTrajectoryPoint()
        trajectory_point.positions = self.gripper_joint_values
        trajectory_point.time_from_start = rospy.Time(5.)
        traj.points = [trajectory_point]
        self.gripper_traj_pub.publish(traj)
        rospy.sleep(3.)

        rospy.loginfo('[PLACE] Moving the arm back')
        self.move_arm(MoveArmGoal.NAMED_TARGET, self.safe_arm_joint_config)

        return 'succeeded'

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
        self.move_arm_client.send_goal(move_arm_goal)
        self.move_arm_client.wait_for_result()
        result = self.move_arm_client.get_result()
        return result

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['place_goal'],
                             output_keys=['place_feedback', 'place_result'])
        self.result = result

    def execute(self, userdata):
        result = PlaceResult()
        result.success = self.result
        userdata.place_result = result
        return 'succeeded'

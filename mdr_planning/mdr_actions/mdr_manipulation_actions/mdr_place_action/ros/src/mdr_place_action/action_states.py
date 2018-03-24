#!/usr/bin/python

import rospy
import smach
import moveit_commander
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from controller_manager_msgs.srv import ListControllers

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
    def __init__(self, timeout=120.0, arm_name='arm',
                 gripper_joint_names=list(),
                 gripper_joint_values=list(),
                 gripper_cmd_topic='/gripper/command'):
        smach.State.__init__(self, input_keys=['place_goal'],
                             output_keys=['place_feedback'],
                             outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.gripper_joint_names = gripper_joint_names
        self.gripper_joint_values = gripper_joint_values
        self.gripper_traj_pub = rospy.Publisher(gripper_cmd_topic, JointTrajectory,
                                                queue_size=10)

        self.arm = moveit_commander.MoveGroupCommander(arm_name)

    def execute(self, userdata):
        feedback = PlaceFeedback()
        feedback.current_state = 'PLACE'
        feedback.message = '[PLACE] moving the arm'
        userdata.place_feedback = feedback

        # we set up the arm group for moving
        self.arm.clear_pose_targets()
        pose = userdata.place_goal.pose
        self.arm.set_pose_reference_frame(pose.header.frame_id)
        self.arm.set_pose_target(pose.pose)

        rospy.loginfo('[PLACE] Planning motion and trying to move arm...')

        # we move the arm
        success = self.arm.go(wait=True)
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

        return 'succeeded'

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

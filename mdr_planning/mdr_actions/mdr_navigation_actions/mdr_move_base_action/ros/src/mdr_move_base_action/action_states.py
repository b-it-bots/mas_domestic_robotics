#!/usr/bin/python

import rospy
import smach
import actionlib
import yaml
import tf
from geometry_msgs.msg import PoseStamped, Quaternion

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from mdr_move_base_action.msg import MoveBaseFeedback, MoveBaseResult

class SetupMoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['move_base_goal'],
                             output_keys=['move_base_feedback'])

    def execute(self, userdata):
        feedback = MoveBaseFeedback()
        feedback.current_state = 'SETUP_MOVE_BASE'
        feedback.message = '[MOVE_BASE] Received a move base request'
        userdata.move_base_feedback = feedback
        return 'succeeded'

class ApproachPose(smach.State):
    def __init__(self, timeout=120.,
                 move_base_server='/move_base',
                 pose_description_file='',
                 pose_frame='map'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['move_base_goal'],
                             output_keys=['move_base_feedback'])
        self.pose = None
        self.move_base_server = move_base_server
        self.pose_description_file = pose_description_file
        self.pose_frame = pose_frame
        self.timeout = timeout

    def execute(self, userdata):
        destination = userdata.move_base_goal.destination_location

        feedback = MoveBaseFeedback()
        feedback.current_state = 'APPROACH_POSE'
        feedback.message = '[MOVE_BASE] Moving base to ' + destination
        userdata.move_base_feedback = feedback

        self.pose = self.convert_pose_name_to_coordinates(destination)
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = self.pose_frame
        pose.pose.position.x = self.pose[0]
        pose.pose.position.y = self.pose[1]

        quat = tf.transformations.quaternion_from_euler(0, 0, self.pose[2])
        pose.pose.orientation = Quaternion(*quat)

        goal = MoveBaseGoal()
        goal.target_pose = pose

        move_base_client = actionlib.SimpleActionClient(self.move_base_server,
                                                        MoveBaseAction)
        move_base_client.wait_for_server()
        move_base_client.send_goal(goal)
        success = move_base_client.wait_for_result()

        if success:
            rospy.loginfo('Pose %s reached successfully' % destination)
            return 'succeeded'
        rospy.logerr('Pose %s could not be reached' % destination)
        return 'failed'

    def convert_pose_name_to_coordinates(self, pose_name):
        stream = file(self.pose_description_file, 'r')
        poses = yaml.load(stream)
        stream.close()
        try:
            coordinates = poses[pose_name]
            return coordinates
        except:
            rospy.logerr('Pose name "%s" does not exist' % (pose_name))
            return None

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['move_base_result'])
        self.result = result

    def execute(self, userdata):
        result = MoveBaseResult()
        result.success = self.result
        userdata.move_base_result = result
        return 'succeeded'

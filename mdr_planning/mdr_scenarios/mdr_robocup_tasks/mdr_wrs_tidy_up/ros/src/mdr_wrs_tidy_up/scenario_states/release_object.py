import numpy as np
import tf
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from mdr_place_action.msg import PlaceAction, PlaceGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class ReleaseObject(ScenarioStateBase):
    tf_listener = None

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'release_object',
                                   save_sm_state=save_sm_state,
                                   input_keys=['grasped_object',
                                               'environment_objects'],
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'release_object')
        self.action_server_name = kwargs.get('action_server_name', 'place_server')
        self.release_timeout = kwargs.get('release_timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.release_target_name = kwargs.get('release_target_name', 'bin_a')
        self.retry_count = 0
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        release_target = userdata.environment_objects[self.release_target_name]

        goal = PlaceGoal()
        goal.pose = self.get_release_pose(release_target, userdata.grasped_object)
        goal.release_on_impact = False

        client = actionlib.SimpleActionClient(self.action_server_name, PlaceAction)
        client.wait_for_server()
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(self.release_timeout)))
        result = client.get_result()
        if result:
            if result.success:
                rospy.loginfo('[%s] Successfully released object', self.state_name)
                return 'succeeded'
            else:
                rospy.logerr('[%s] Could not release object', self.state_name)
        else:
            rospy.logerr('[%s] Object not released within %f seconds; giving up',
                         self.state_name, self.release_timeout)
            client.cancel_all_goals()

        if self.retry_count == self.number_of_retries:
            rospy.logerr('[%s] Could not release object after retrying; giving up', self.state_name)
            self.retry_count = 0
            return 'failed_after_retrying'

        rospy.loginfo('[%s] Retrying to release object', self.state_name)
        self.retry_count += 1
        return 'succeeded'

    def get_release_pose(self, release_target, grasped_object):
        candidate_pose = PoseStamped()
        candidate_pose.header.stamp = rospy.Time(0)
        candidate_pose.header.frame_id = release_target.pose.header.frame_id

        candidate_pose.pose.position.x = release_target.pose.pose.position.x
        candidate_pose.pose.position.y = release_target.pose.pose.position.y
        candidate_pose.pose.position.z = release_target.pose.pose.position.z + \
                                         (release_target.dimensions.vector.z / 2)
        candidate_pose.pose.position.z += 0.1

        # use top-down orientation for throwing
        gripper_pose = PoseStamped()
        gripper_pose.header.frame_id = 'base_link'
        gripper_pose.header.stamp = rospy.Time(0)

        gripper_quaternion = tf.transformations.quaternion_from_euler(np.pi, 0., 0.)
        gripper_pose.pose.orientation.x = gripper_quaternion[0]
        gripper_pose.pose.orientation.y = gripper_quaternion[1]
        gripper_pose.pose.orientation.z = gripper_quaternion[2]
        gripper_pose.pose.orientation.w = gripper_quaternion[3]

        gripper_pose_in_target_frame = self.tf_listener.transformPose(release_target.pose.header.frame_id, gripper_pose)
        candidate_pose.pose.orientation = gripper_pose_in_target_frame.pose.orientation
        return candidate_pose

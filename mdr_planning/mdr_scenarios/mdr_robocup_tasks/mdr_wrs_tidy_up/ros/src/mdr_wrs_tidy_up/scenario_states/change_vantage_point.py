import random

import tf
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class ChangeVantagePoint(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'change_vantage_point',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'],
                                   input_keys=[])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'change_vantage_point')
        self.action_server_name = kwargs.get('action_server_name', 'move_base_server')
        self.robot_pose_topic = kwargs.get('robot_pose_topic', '/hsrb/base_pose')
        self.timeout = kwargs.get('timeout', 120.)
        self.debug = kwargs.get('debug', False)
        self.tf_listener = tf.TransformListener()
        self.__init_ros_components()

        self.robot_pose_msg = None

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        self.move_base_client.wait_for_server()
        goal = MoveBaseGoal()
        goal.goal_type = MoveBaseGoal.POSE

        try:
            goal.pose = self.robot_pose_msg
        except AttributeError:
            rospy.logerr('[%s] Could not obtain current pose', self.state_name)
            if self.retry_count == self.number_of_retries:
                rospy.logerr('[%s] Failed even after retrying; giving up',
                             self.state_name)
                self.retry_count = 0
                return 'failed_after_retrying'
            self.retry_count += 1
            return 'failed'

        goal.pose = self.transform_pose(goal.pose)
        goal.pose.pose.position.x += random.gauss(0, 0.1)

        self.move_base_client.send_goal(goal)
        self.move_base_client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
        result = self.move_base_client.get_result()
        if result:
            if result.success:
                rospy.loginfo('[%s] Successfully changed vantage point', self.state_name)
            else:
                rospy.logerr('[%s] Could not change vantage point', self.state_name)
                if self.retry_count == self.number_of_retries:
                    rospy.logerr('[%s] Failed even after retrying; giving up',
                                 self.state_name)
                    self.retry_count = 0
                    return 'failed_after_retrying'
                self.retry_count += 1
                return 'failed'
        else:
            rospy.logerr('[%s] Could not move within %f seconds; giving up',
                         self.state_name, self.timeout)
            self.move_base_client.cancel_all_goals()
        return 'succeeded'

    def pose_callback(self, pose_msg):
        self.robot_pose_msg = pose_msg

    def transform_pose(self, pose_msg):
        transformed_pose = None
        while not rospy.is_shutdown():
            try:
                transformed_pose = self.tf_listener.transformPose('map', pose_msg)
                break
            except:
                continue

        return transformed_pose

    def __init_ros_components(self):
        '''Initialises:
        * a move base action client
        * a subscriber for the robot's pose (in map frame)
        '''
        rospy.loginfo('[%s] Initialising client for action %s',
                      self.state_name, self.action_server_name)
        self.move_base_client = actionlib.SimpleActionClient(self.action_server_name,
                                                                    MoveBaseAction)
        rospy.loginfo('[%s] Client for action %s initialised',
                      self.state_name, self.action_server_name)

        rospy.loginfo('[%s] Creating subscriber for topic %s',
                      self.state_name, self.robot_pose_topic)
        self.obstacle_sub = rospy.Subscriber(self.robot_pose_topic,
                                             PoseStamped, self.pose_callback)
        rospy.sleep(0.5)
        rospy.loginfo('[%s] Subscriber for topic %s created',
                      self.state_name, self.robot_pose_topic)

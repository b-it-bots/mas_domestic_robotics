import rospy
import actionlib

from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase
import tf
from geometry_msgs.msg import PoseStamped
class MoveBaseToPerson(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'move_base',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'],
                                   input_keys=['person_pose'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'move_base')
        self.action_server_name = kwargs.get('action_server_name', 'move_base_server')
        self.person_pose = kwargs.get('person_pose', None)
        self.timeout = kwargs.get('timeout', 120.)
        self.debug = kwargs.get('debug', False)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.tf_listener = tf.TransformListener()
        self.target_person_pose_map = rospy.Publisher('target_person_pose_map', PoseStamped, queue_size=1)
        self.target_person_pose_base_link = rospy.Publisher('target_person_pose_base_link', PoseStamped, queue_size=1)
        self.target_person_pose_base_link_with_offset = rospy.Publisher('target_person_pose_base_link_with_offset', PoseStamped, queue_size=1)

    def execute(self, userdata):
        person_pose = None
        if self.person_pose is None:
            person_pose = userdata.person_pose
            rospy.loginfo("[move_base_to_person] Using userdata's person pose %s", person_pose)
        else:
            person_pose = self.person_pose
            rospy.loginfo("[move_base_to_person] Using predefined person_pose %s", person_pose)

        client = actionlib.SimpleActionClient(self.action_server_name, MoveBaseAction)
        client.wait_for_server()
        goal = MoveBaseGoal()
        goal.goal_type = MoveBaseGoal.POSE

        person_pose.header.stamp = rospy.Time().now()

        self.target_person_pose_map.publish(person_pose)
        rospy.loginfo("[move_base_to_person] Published person pose in map frame %s", person_pose)
        
        person_pose.header.stamp = rospy.Time(0,0)
        person_pose_in_base_link = self.tf_listener.transformPose('base_link', person_pose)
        person_pose = person_pose_in_base_link        
        person_pose.header.stamp = rospy.Time().now()

        self.target_person_pose_base_link.publish(person_pose)
        rospy.loginfo("[move_base_to_person] Published person pose in base_link frame %s", person_pose)

        person_pose.pose.position.x -= 1.0
        person_pose.pose.position.y -= 1.0
        person_pose.pose.position.z = 0.0
        
        self.target_person_pose_base_link_with_offset.publish(person_pose)
        rospy.loginfo("[move_base_to_person] Published person pose in base_link frame with offset %s", person_pose)
        
        goal.pose.header.frame_id = person_pose.header.frame_id
        goal.pose.pose.position.x = person_pose.pose.position.x
        goal.pose.pose.position.y = person_pose.pose.position.y
        rospy.loginfo("[move_base_to_person] Pose after pre-defined offset  %s", goal.pose)
        client.send_goal(goal)
        client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
        result = client.get_result()
        if result:
            if result.success:
                rospy.loginfo('[move_base_to_person] [%s] Successfully moved to %s',
                                self.state_name, goal.pose)
            else:
                rospy.logerr('[%s] Could not move to %s', self.state_name, goal.pose)
                if self.retry_count == self.number_of_retries:
                    rospy.logerr('[%s] Could not move to %s even after retrying; giving up',
                                    self.state_name, goal.pose)
                    self.retry_count = 0
                    return 'failed_after_retrying'
                self.retry_count += 1
                return 'failed'
        else:
            rospy.logerr('[%s] Could not move to %s within %f seconds; giving up',
                            self.state_name, goal.pose, self.timeout)
            client.cancel_all_goals()
        return 'succeeded'

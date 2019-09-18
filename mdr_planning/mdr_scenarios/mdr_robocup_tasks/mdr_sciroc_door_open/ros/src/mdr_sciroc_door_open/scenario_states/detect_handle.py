import rospy
from geometry_msgs.msg import PoseStamped
from mas_execution_manager.scenario_state_base import ScenarioStateBase


class DetectHandle(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'detect_handle',
                                   save_sm_state=save_sm_state,
                                   output_keys=['handle_pose'],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'detect_handle')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)

    def execute(self, userdata):
        self.say('trying to detect handle')
        rospy.sleep(5.0)
        handle_pose = PoseStamped()
        handle_pose.header.frame_id = 'base_link'
        handle_pose.header.stamp = rospy.Time.now()

        handle_pose.pose.position.x = 0.418
        handle_pose.pose.position.y = 0.078
        handle_pose.pose.position.z = 0.842

        handle_pose.pose.orientation.x = 0.758
        handle_pose.pose.orientation.y = 0.000
        handle_pose.pose.orientation.z = 0.8
        handle_pose.pose.orientation.w = 0.000
        userdata.handle_pose = handle_pose
        return 'succeeded'

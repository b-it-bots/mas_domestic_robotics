import rospy
from mas_execution_manager.scenario_state_base import ScenarioStateBase


class OpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'open_door',
                                   save_sm_state=save_sm_state, input_keys=['handle_pose'],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)

    def execute(self, userdata):
        self.say('trying to open door')
        rospy.loginfo('handle pose: {} {} {}'.format(userdata.handle_pose.pose.position.x,
                                                     userdata.handle_pose.pose.position.y,
                                                     userdata.handle_pose.pose.position.z))
        rospy.sleep(5.0)
        return 'failed'


class AskToOpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'ask_to_open_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'ask_to_open_door')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)

    def execute(self, userdata):
        self.say('I cannot open the door myself, can you please open the door')
        rospy.sleep(15.0)
        return 'succeeded'

import rospy

from mas_execution_manager.scenario_state_base import ScenarioStateBase
from std_msgs.msg import Bool


class DetectPerson(ScenarioStateBase):

    DOOR_STATUS_TOPIC = '/mcr_perception/door_status/door_status'

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'detect_person',
                                   save_sm_state=save_sm_state,
                                   input_keys=[],
                                   output_keys=[],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        # Get parameters
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'detect_person')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 30) # not used right now

        self.person_in_front = False
        self.person_status_sub = rospy.Subscriber(DetectPerson.DOOR_STATUS_TOPIC, Bool, self.update_person_in_front)

    def execute(self, userdata):
        self.say('Please step in front of me')
        rospy.sleep(1)

        rospy.loginfo('Waiting for person to step in front...')
        while not self.person_in_front:
            rospy.sleep(0.5)
        rospy.loginfo('Someone stepped in front of me!')

        return 'succeeded'

    def update_person_in_front(self, msg):
        self.person_in_front = not msg.data

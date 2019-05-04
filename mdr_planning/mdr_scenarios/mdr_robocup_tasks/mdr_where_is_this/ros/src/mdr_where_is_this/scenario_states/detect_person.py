import rospy
import actionlib

from mas_execution_manager.scenario_state_base import ScenarioStateBase
from std_msgs.msg import Bool


class DetectPerson(ScenarioStateBase):

    DOOR_STATUS_TOPIC = '/mcr_perception/door_status/door_status'

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'detect_person',
                                   save_sm_state=save_sm_state,
                                   input_keys=[],
                                   output_keys=[],
                                   outcomes=['succeeded', 'failed'])

        self.person_in_front = False
        self.person_status_sub = rospy.Subscriber(DOOR_STATUS_TOPIC, Bool, self.update_person_in_front)

    def execute(self, userdata):
        self.say('Please step in front of me')
        rospy.sleep(1)

        rospy.loginfo('Waiting for person to step in front...')
        while not self.person_in_front:
            rospy.sleep(0.5)
        rospy.loginfo('Someone stepped in front of me!')

        return 'succeeded'

    def update_door_open(self, msg):
        self.person_in_front = !msg.data

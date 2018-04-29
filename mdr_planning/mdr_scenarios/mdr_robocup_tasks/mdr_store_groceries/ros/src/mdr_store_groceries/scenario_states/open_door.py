import rospy

from std_msgs.msg import String

from mdr_execution_manager.scenario_state_base import ScenarioStateBase

class OpenDoor(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'open_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'waiting', 'failed'],
                                   output_keys=['command'])
        self.timeout = rospy.Duration.from_sec(kwargs.get('timeout', 10.))

        self.start_time = rospy.Time.now()
        self.restart_state = False
        self.asked_for_door_opening = False

    def execute(self, userdata):
        if not self.asked_for_door_opening:
            self.say('Please open the cupboard door for me')
            self.asked_for_door_opening = True

        if (rospy.Time.now() - self.start_time) < self.timeout:
            return 'waiting'
        return 'succeeded'

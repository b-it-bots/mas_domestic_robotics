import rospy

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class Say(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'say',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.timeout = rospy.Duration.from_sec(kwargs.get('timeout', 10.))
        self.sentence = kwargs.get('sentence', '')
        self.sound_topic = kwargs.get('sound_topic', '/say')
        self.waiting_time_sec = kwargs.get('waiting_time_sec', 5.)

    def execute(self, userdata):
        self.say(self.sentence)
        rospy.sleep(self.waiting_time_sec)
        return 'succeeded'

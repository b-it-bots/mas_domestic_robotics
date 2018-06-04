import rospy

from std_msgs.msg import String

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class WaitForName(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'wait_for_name',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.timeout = rospy.Duration.from_sec(kwargs.get('timeout', 10.))
        self.speech_topic = kwargs.get('speech_topic', '/recognized_speech')
        self.robot_name = kwargs.get('name', 'bot')
        self.cmd_received = False
        self.speech_sub = rospy.Subscriber(self.speech_topic, String, self.get_speech_cmd)

    def execute(self, userdata):
        while not self.cmd_received:
            rospy.sleep(0.1)
        self.cmd_received = False
        return 'succeeded'

    def get_speech_cmd(self, speech_msg):
        cmd = speech_msg.data
        if cmd.lower().find(self.robot_name) != -1:
            self.cmd_received = True

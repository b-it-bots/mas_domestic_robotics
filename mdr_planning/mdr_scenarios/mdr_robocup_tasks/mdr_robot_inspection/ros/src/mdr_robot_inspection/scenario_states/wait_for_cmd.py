import rospy

from std_msgs.msg import String

from mdr_robot_inspection.scenario_states.scenario_state_base import ScenarioStateBase

class WaitForCmd(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'wait_for_cmd',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'waiting', 'failed'],
                                   output_keys=['command'])
        self.timeout = rospy.Duration.from_sec(kwargs.get('timeout', 120.))
        self.state_check_rate = rospy.Rate(5)

        self.say_topic = kwargs.get('say_topic', '')
        self.say_enabled = self.say_topic != ''
        self.say_pub = rospy.Publisher(self.say_topic, String, latch=True, queue_size=1)

        self.speech_sub = rospy.Subscriber(kwargs.get('speech_topic', '/recognized_speech'),
                                           String, self.command_cb)
        self.command = None
        self.start_time = rospy.Time.now()
        self.restart_state = False

    def execute(self, userdata):
        if self.restart_state:
            self.start_time = rospy.Time.now()
            self.restart_state = False

        if (rospy.Time.now() - self.start_time) < self.timeout:
            if self.command:
                rospy.loginfo('Received command: %s' % self.command)
                self.say(self.say_enabled, self.say_pub,
                         'Received command ' + self.command)
                userdata.command = self.command
                self.restart_state = True
                return 'succeeded'

            self.state_check_rate.sleep()
            rospy.loginfo('Waiting for command')
            return 'waiting'

        rospy.loginfo('wait_for_cmd timed out')
        self.restart_state = True
        return 'failed'

    def command_cb(self, msg):
        self.command = msg.data

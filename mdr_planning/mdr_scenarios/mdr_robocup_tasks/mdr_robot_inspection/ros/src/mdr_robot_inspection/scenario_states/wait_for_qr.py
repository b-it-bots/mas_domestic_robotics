import rospy

from std_msgs.msg import String

from mdr_robot_inspection.scenario_states.scenario_state_base import ScenarioStateBase

class WaitForQR(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'wait_for_qr',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'waiting', 'failed'],
                                   output_keys=['command'])
        self.timeout = rospy.Duration.from_sec(kwargs.get('timeout', 120.))
        self.state_check_rate = rospy.Rate(5)
        self.qr_code_sub = rospy.Subscriber(kwargs.get('qr_topic',
                                                       '/mcr_perception/qr_reader/output'),
                                            String, self.register_qr_code)
        self.qr_message = None

        self.say_topic = kwargs.get('say_topic', '')
        self.say_enabled = self.say_topic != ''
        self.say_pub = rospy.Publisher(self.say_topic, String, latch=True, queue_size=1)

        self.start_time = rospy.Time.now()

    def execute(self, userdata):
        self.start_time = rospy.Time.now()
        self.say(self.say_enabled, self.say_pub, 'Please show me a QR code')
        while (rospy.Time.now() - self.start_time) < self.timeout and self.qr_message is None:
            if self.qr_message and "continue" in self.qr_message.lower():
                rospy.loginfo('QR message: %s' % self.qr_message)
                self.say(self.say_enabled, self.say_pub, 'Continuing operation')
                userdata.command = self.qr_message
                break

            self.state_check_rate.sleep()
            rospy.loginfo('Waiting for QR code')

        if self.qr_message is None:
            rospy.loginfo('wait_for_qr timed out')
        else:
            self.qr_message = None
            return 'succeeded'
        return 'failed'

    def register_qr_code(self, msg):
        self.qr_message = msg.data

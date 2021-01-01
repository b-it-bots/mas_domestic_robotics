import rospy
from std_msgs.msg import Empty
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class PerformDisinfection(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'perform_disinfection',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'perform_disinfection')
        self.disinfection_cmd_topic = kwargs.get('disinfection_cmd_topic',
                                                 '/disinfect_cmd')
        self.disinfection_duration = kwargs.get('disinfection_duration', 20.)

        self.disinfection_cmd_pub = None
        try:
            self.disinfection_cmd_pub = rospy.Publisher(self.disinfection_cmd_topic,
                                                        Empty, queue_size=1)
        except Exception as exc:
            rospy.logerr('[send_disinfection_command] Could not create a disinfection command publisher')
            rospy.logerr(str(exc))

    def execute(self, userdata):
        self.disinfection_cmd_pub.publish(Empty())
        self.say('Please hold out your hand to my friend Kinova. You will then get some hand sanitizer.')
        rospy.sleep(self.disinfection_duration)
        return 'succeeded'

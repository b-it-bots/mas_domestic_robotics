import rospy
from std_msgs.msg import String
from actionlib import SimpleActionClient
from mdr_enter_door_action.msg import EnterDoorAction, EnterDoorGoal
from mdr_robot_inspection.scenario_states.scenario_state_base import ScenarioStateBase

class Enter(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'enter_door',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                                   output_keys=['enter_door_feedback'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

        self.say_topic = kwargs.get('say_topic', '')
        self.say_enabled = self.say_topic != ''
        self.say_pub = rospy.Publisher(self.say_topic, String, latch=True, queue_size=1)

        self.enter_door_server = kwargs.get('enter_action_server', 'enter_door_server')
        self.enter_action_client = SimpleActionClient(self.enter_door_server,
                                                      EnterDoorAction)
        self.enter_action_client.wait_for_server()

    def execute(self, userdata):
        goal = EnterDoorGoal()
        rospy.loginfo('[ENTER_DOOR] Calling door entering action')
        self.say(self.say_enabled, self.say_pub, 'Entering door')
        self.enter_action_client.send_goal(goal)
        duration = rospy.Duration.from_sec(self.timeout)
        success = self.enter_action_client.wait_for_result(duration)
        if success:
            rospy.loginfo('Entered successfully')
            return 'succeeded'
        else:
            rospy.logerr('Entering failed')
            self.say(self.say_enabled, self.say_pub, 'Could not enter door')
            if self.retry_count == self.number_of_retries:
                self.say(self.say_enabled, self.say_pub, 'Aborting operation')
                return 'failed_after_retrying'
            return 'failed'

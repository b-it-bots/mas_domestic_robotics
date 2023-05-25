import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from mdr_listen_action.msg import ListenAction, ListenGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class AskName(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'ask_name',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['person_name'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'ask_name')
        self.timeout = 10.

        # wait for listen action server
        self.listen_client = actionlib.SimpleActionClient('listen_server', ListenAction)
        listen_wait_result = self.listen_client.wait_for_server(timeout=rospy.Duration(self.timeout))
        if not listen_wait_result:
            raise RuntimeError('Failed to wait for "listen_server" action')

    def execute(self, userdata):
        sentence = 'Please tell me your first name'
        rospy.loginfo('Saying {0}'.format(sentence))
        self.say(sentence)

        goal = ListenGoal()
        self.listen_client.send_goal(goal)
        self.listen_client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
        listen_state = self.listen_client.get_state()
        listen_result = self.listen_client.get_result()

        if listen_state == GoalStatus.SUCCEEDED:
            rospy.loginfo('Understood: {}'.format(listen_result.message))
            userdata.person_name = listen_result.message
            return 'succeeded'
        rospy.logerr('Could not get input, listen action returned None')
        return 'failed'

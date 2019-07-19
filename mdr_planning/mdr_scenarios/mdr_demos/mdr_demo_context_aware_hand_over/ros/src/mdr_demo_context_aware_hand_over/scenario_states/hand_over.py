import rospy
import actionlib

from mdr_hand_over_action.msg import HandOverAction, HandOverGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class HandOver(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'hand_over',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                                   input_keys=['posture'])
        self.sm_id = kwargs.get('sm_id', 'mdr_demo_context_aware_hand_over')
        self.action_server = kwargs.get('action_server', 'hand_over')
        self.obstacle_present = kwargs.get('obstacle_present', False)
        self.timeout = kwargs.get('timeout', 120.)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.client = actionlib.SimpleActionClient(self.action_server, HandOverAction)
        self.client.wait_for_server(rospy.Duration(10.))

    def execute(self, userdata):
        goal = HandOverGoal()
        goal.posture_type = userdata.posture
        goal.obstacle = self.obstacle_present

        # calling the actionlib server and waiting for the execution to end
        rospy.loginfo('[hand_over] Sending action lib goal to {0}'.format(self.action_server))
        self.say('Handing item over to a {0} person'.format(userdata.posture))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
        result = self.client.get_result()

        if result and result.success:
            rospy.loginfo('Successfully handed object over')
            return 'succeeded'
        else:
            rospy.logerr('Could not hand object over')
            self.say('Could not hand object over')
            if self.retry_count == self.number_of_retries:
                self.say('Aborting operation')
                return 'failed_after_retrying'
            self.retry_count += 1
            return 'failed'

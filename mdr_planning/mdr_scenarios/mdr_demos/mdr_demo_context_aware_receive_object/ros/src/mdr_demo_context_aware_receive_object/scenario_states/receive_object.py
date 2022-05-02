import rospy
import actionlib

from mdr_receive_object_action.msg import ReceiveObjectAction, ReceiveObjectGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class ReceiveObject(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receive_object',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                                   input_keys=['posture', 'person_pose'])
        self.sm_id = kwargs.get('sm_id', 'mdr_demo_context_aware_receive_object')
        self.action_server = kwargs.get('action_server', 'receive_object_server')
        self.timeout = kwargs.get('timeout', 120.)
        self.context_aware = kwargs.get('context_aware', True)
        self.reception_detection = kwargs.get('reception_detection', True)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.client = actionlib.SimpleActionClient(self.action_server, ReceiveObjectAction)
        self.client.wait_for_server(rospy.Duration(10.))

    def execute(self, userdata):
        goal = ReceiveObjectGoal()
        goal.posture_type = userdata.posture
        goal.context_aware = self.context_aware
        goal.reception_detection = self.reception_detection
        goal.person_pose = userdata.person_pose

        # calling the actionlib server and waiting for the execution to end
        rospy.loginfo('[receive_object] Sending action lib goal to {0}'.format(self.action_server))
        self.say('Receiving object from a {0} person'.format(userdata.posture))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
        result = self.client.get_result()

        if result and result.success:
            rospy.loginfo('Received object')
            return 'succeeded'
        else:
            rospy.logerr('Could not receive object')
            self.say('Object not received')
            if self.retry_count == self.number_of_retries:
                self.say('Aborting operation')
                return 'failed_after_retrying'
            self.retry_count += 1
            return 'failed'

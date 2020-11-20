import rospy
import time
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class FindPeople(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'find_people',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'find_people')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.debug = kwargs.get('debug', False)
        self.perform_recognition = kwargs.get('perform_recognition', False)
        self.retry_count = 0
        self.timeout = 120.

    def execute(self, userdata):
        rospy.loginfo('[find_people] Trying to find people')
        dispatch_msg = self.get_dispatch_msg()
        self.action_dispatch_pub.publish(dispatch_msg)

        self.executing = True
        self.succeeded = False
        start_time = time.time()
        duration = 0.
        while self.executing and duration < self.timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.succeeded:
            rospy.loginfo('[find_people] Successfully found people')
            if self.debug: self.say('Successfully found people')
            return 'succeeded'

        rospy.loginfo('Could not find people')
        if self.debug: self.say('Could not find people')
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('[find_people] Failed to find people')
            if self.debug: self.say('Aborting operation')
            return 'failed_after_retrying'
        rospy.loginfo('[find_people] Retrying to find people')
        self.retry_count += 1
        return 'failed'

    def get_dispatch_msg(self):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = self.action_name

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'bot'
        arg_msg.value = self.robot_name
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'perform_recognition'
        arg_msg.value = str(self.perform_recognition)
        dispatch_msg.parameters.append(arg_msg)

        return dispatch_msg

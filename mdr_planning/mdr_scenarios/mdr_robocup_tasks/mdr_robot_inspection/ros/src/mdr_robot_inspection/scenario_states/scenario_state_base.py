import rospy
import smach
from std_msgs.msg import String

from mongodb_store.message_store import MessageStoreProxy

from mdr_monitoring_msgs.msg import ExecutionState

class ScenarioStateBase(smach.State):
    def __init__(self, action_name, outcomes,
                 input_keys=list(), output_keys=list(),
                 save_sm_state=False):
        smach.State.__init__(self, outcomes=outcomes,
                             input_keys=input_keys,
                             output_keys=output_keys)
        self.sm_id = ''
        self.state_name = ''
        self.action_name = action_name
        self.save_sm_state = save_sm_state
        self.retry_count = 0
        self.msg_store_client = MessageStoreProxy()

    def execute(self, userdata):
        pass

    def save_current_state(self):
        execution_state_msg = ExecutionState()
        execution_state_msg.stamp = rospy.Time.now()
        execution_state_msg.state_machine = self.sm_id
        execution_state_msg.state = self.state_name
        try:
            self.msg_store_client.insert_named('current_state', execution_state_msg)
        except:
            rospy.logerr('Error while saving current state')

    def say(self, say_enabled, publisher, sentence):
        if say_enabled:
            say_msg = String()
            say_msg.data = sentence
            publisher.publish(say_msg)

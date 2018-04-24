import rospy
import smach
from std_msgs.msg import String

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
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
        self.executing = False
        self.succeeded = False

        self.action_dispatch_pub = rospy.Publisher('/kcl_rosplan/action_dispatch',
                                                   plan_dispatch_msgs.ActionDispatch,
                                                   queue_size=1)

        rospy.Subscriber('/kcl_rosplan/action_feedback',
                         plan_dispatch_msgs.ActionFeedback,
                         self.get_action_feedback)

        rospy.wait_for_service('/kcl_rosplan/get_current_knowledge')
        self.attribute_fetching_client = rospy.ServiceProxy('/kcl_rosplan/get_current_knowledge',
                                                            rosplan_srvs.GetAttributeService)

        self.msg_store_client = MessageStoreProxy()
        self.robot_name = ''
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'robot_name'
        result = self.attribute_fetching_client(request)
        for item in result.attributes:
            for param in item.values:
                if param.key == 'bot':
                    self.robot_name = param.value
                    break
            break

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

    def get_dispatch_msg(self):
        pass

    def get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.action_name:
            self.executing = False
            self.succeeded = msg.status == 'action achieved'

    def say(self, say_enabled, publisher, sentence):
        if say_enabled:
            say_msg = String()
            say_msg.data = sentence
            publisher.publish(say_msg)

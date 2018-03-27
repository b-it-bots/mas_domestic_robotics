import time
import rospy
import smach

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs

class ScenarioStateBase(smach.State):
    def __init__(self, action_name, outcomes):
        smach.State.__init__(self, outcomes)
        self.action_name = action_name
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

    def get_dispatch_msg(self):
        pass

    def get_action_feedback(self, msg):
        if msg.information and msg.information[0].key == 'action_name' and \
        msg.information[0].value == self.action_name:
            self.executing = False
            self.succeeded = msg.status == 'action achieved'

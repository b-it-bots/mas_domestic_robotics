#!/usr/bin/env python

import abc

import rospy
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
import diagnostic_msgs.msg as diag_msgs

class ActionClientBase(object):
    def __init__(self):
        self.action_success_msg = 'action achieved'
        self.action_failure_msg = 'action failed'
        self.action_id = -1

        self.robot_name = None

        self.action_name = rospy.get_param('~action_name', '')
        self.action_name = self.action_name.lower()

        self.action_server_name = rospy.get_param('~server_name', '')
        self.action_timeout = rospy.get_param('~action_timeout', 15.)

        self.knowledge_update_client = rospy.ServiceProxy('knowledge_update_service',
                                                          rosplan_srvs.KnowledgeUpdateService)

        self.attribute_fetching_client = rospy.ServiceProxy('knowledge_update_service',
                                                            rosplan_srvs.GetAttributeService)

        rospy.Subscriber('action_dispatch_topic',
                         plan_dispatch_msgs.ActionDispatch,
                         self.call_action)

        self.feedback_pub = rospy.Publisher('action_feedback_topic',
                                            plan_dispatch_msgs.ActionFeedback,
                                            queue_size=1)

    @abc.abstractmethod
    def call_action(self, msg):
        pass

    @abc.abstractmethod
    def get_action_message(self, rosplan_action_msg):
        return None

    @abc.abstractmethod
    def update_knowledge_base(self, source_location, destination_location):
        pass

    @abc.abstractmethod
    def send_action_feedback(self, success):
        msg = plan_dispatch_msgs.ActionFeedback()
        msg.action_id = self.action_id
        if success:
            msg.status = self.action_success_msg
        else:
            msg.status = self.action_failure_msg

        action_name_kvp = diag_msgs.KeyValue()
        action_name_kvp.key = 'action_name'
        action_name_kvp.value = self.action_name
        msg.information.append(action_name_kvp)

        self.feedback_pub.publish(msg)

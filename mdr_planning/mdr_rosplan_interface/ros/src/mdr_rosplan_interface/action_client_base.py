#!/usr/bin/env python

import abc

import rospy
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface

class ActionClientBase(object):
    '''An abstract base class for knowledge-enabled action clients.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self):
        # unique action ID
        self.action_id = -1

        # name of the robot on which the client is spawned
        self.robot_name = None

        # name of the action (converted to lowercase)
        self.action_name = rospy.get_param('~action_name', '')
        self.action_name = self.action_name.lower()

        # name of the action server
        self.action_server_name = rospy.get_param('~server_name', '')

        # timeout for action calls
        self.action_timeout = rospy.get_param('~action_timeout', 15.)

        # knowledge base interface instance
        self.kb_interface = DomesticKBInterface()

        # subscriber for dispatched actions
        rospy.Subscriber('action_dispatch_topic',
                         plan_dispatch_msgs.ActionDispatch,
                         self.call_action)

        # action feedback publisher
        self.feedback_pub = rospy.Publisher('action_feedback_topic',
                                            plan_dispatch_msgs.ActionFeedback,
                                            queue_size=1)

    @abc.abstractmethod
    def call_action(self, msg):
        '''Abstract callback for the dispatched action subscriber.
        Only reacts to request to "self.action_name"; ignores all other requests.

        Keyword arguments:
        msg -- a rosplan_dispatch_msgs.msg.ActionDispatch instance

        '''
        pass

    @abc.abstractmethod
    def get_action_message(self, rosplan_action_msg):
        '''Abstract method for converting the message to an action request.
        Returns an actionlib goal instance for the action.

        Keyword arguments:
        rosplan_action_msg -- a rosplan_dispatch_msgs.msg.ActionDispatch instance

        '''
        return None

    @abc.abstractmethod
    def update_knowledge_base(self):
        '''Abstract method for updating the knowledge base after
        the successful completion of an action.
        '''
        pass

    def send_action_feedback(self, success):
        '''Publishes a rosplan_dispatch_msgs.msg.ActionFeedback message
        based on the result of the action execution.

        Keyword arguments:
        success -- a Boolean indicating whether the action was successfully executed

        '''
        msg = plan_dispatch_msgs.ActionFeedback()
        msg.action_id = self.action_id
        if success:
            msg.status = plan_dispatch_msgs.ActionFeedback.ACTION_SUCCEEDED_TO_GOAL_STATE
        else:
            msg.status = plan_dispatch_msgs.ActionFeedback.ACTION_FAILED

        action_name_kvp = diag_msgs.KeyValue()
        action_name_kvp.key = 'action_name'
        action_name_kvp.value = self.action_name
        msg.information.append(action_name_kvp)

        self.feedback_pub.publish(msg)

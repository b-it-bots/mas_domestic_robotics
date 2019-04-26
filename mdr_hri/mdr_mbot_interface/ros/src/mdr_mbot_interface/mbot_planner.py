#!/usr/bin/env python

import rospy
import copy
import rospkg
from std_msgs.msg import String
import diagnostic_msgs.msg as diag_msgs
import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
from mbot_nlu.msg import Slot, ActionSlot, ActionSlotArray

class MbotPlanner(object):
    '''
    Planner class that publishes the corresponding action depending on the
    received interpretation
    '''
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', '')
        self.action_completed = False
        self.action_failed = False

        self.action_dispatch_pub = rospy.Publisher('/kcl_rosplan/action_dispatch',
                                                   plan_dispatch_msgs.ActionDispatch,
                                                   queue_size=1)

        # Subscribe to the topic publishing the interpretation of the ocmmand
        rospy.Subscriber('~input_interpretation',
                         ActionSlotArray,
                         self.interpretationCallback)
        rospy.Subscriber('/kcl_rosplan/action_feedback',
                         plan_dispatch_msgs.ActionFeedback,
                         self.action_feedback_cb)

        # to publish the recognition
        # self.pub_sentence_recog = rospy.Publisher('~output_recognition', ActionSlotArray, queue_size=1)
        # flag to indicate that a text was received and needs to be processed
        self.interpretation_received = False
        # to store the sentece that will be received in the callback
        self.received_interpretation = None
        # inform the user that the node has initialized
        rospy.loginfo("Planner is ready to receive actions")

    def interpretationCallback(self, msg):
        self.received_interpretation = msg
        self.interpretation_received = True

    def process_interpretation(self):
        '''
        Function that publish the right action depending on the interpretation received
        '''
        for action in self.received_interpretation.sentence_recognition:
            dispatch_msg = self.get_dispatch_msg(action)
            rospy.loginfo('\033[1;36m[process_interpretation] Dispatching action {0}\033[0;37m'.format(dispatch_msg.name))
            self.action_dispatch_pub.publish(dispatch_msg)
            while not self.action_completed and not self.action_failed:
                rospy.sleep(0.1)

            if self.action_failed:
                rospy.logerr('[process_interpretation] {0} could not be executed successfully; aborting plan'.format(action.intention))
                self.action_failed = False
                break

            self.action_completed = False
        rospy.loginfo('\033[1;36m[process_interpretation] Done dispatching actions\033[0;37m')

    def get_dispatch_msg(self, action):
       dispatch_msg = plan_dispatch_msgs.ActionDispatch()

       arg_msg = diag_msgs.KeyValue()
       arg_msg.key = 'bot'
       arg_msg.value = self.robot_name
       dispatch_msg.parameters.append(arg_msg)

       if action.intention == 'go':
           dispatch_msg.name = 'move_base'
           for slot in action.slots:
               arg_msg = diag_msgs.KeyValue()
               if slot.type == 'destination':
                   arg_msg.key = 'to'
               arg_msg.value = slot.data
               dispatch_msg.parameters.append(arg_msg)
       elif action.intention == 'take':
           for slot in action.slots:
               arg_msg = diag_msgs.KeyValue()
               if slot.type == 'object':
                   arg_msg.key = 'obj'
               elif slot.type == 'source':
                   dispatch_msg.name = 'pick'
                   arg_msg.key = 'surface'
               elif slot.type == 'destination':
                   dispatch_msg.name = 'place'
                   arg_msg.key = 'surface'
               arg_msg.value = slot.data
               dispatch_msg.parameters.append(arg_msg)

       return dispatch_msg

    def action_feedback_cb(self, msg):
        if msg.status == 'action achieved':
            self.action_completed = True
        elif msg.status == 'action failed':
            self.action_failed = True

    def wait_for_interpretation(self):
        while not rospy.is_shutdown():
            if self.interpretation_received == True:
                # lower flag
                self.interpretation_received = False


                # recognize intention
                self.process_interpretation()
                # action_slot_array_msg = ActionSlotArray()
                # for phrase in recognized_sentence:
                #     # create empty msg
                #     action_slot_msg = ActionSlot()
                #     # fill msg
                #     action_slot_msg.intention = phrase[0]
                #     for slot in phrase[1]:
                #         action_slot_msg.slots.append( Slot(type=slot[0], data=slot[1]) )
                #     # append each single action_slot to from the array
                #     action_slot_array_msg.sentence_recognition.append(copy.deepcopy(action_slot_msg))
                # # publish recognition
                # self.pub_sentence_recog.publish(action_slot_array_msg)
            # sleep to control the frequency of this node
            rospy.sleep(0.1)

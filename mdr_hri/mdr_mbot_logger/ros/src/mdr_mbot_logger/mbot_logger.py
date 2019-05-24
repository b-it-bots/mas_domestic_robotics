#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from mbot_nlu.msg import Slot, ActionSlotArray
import datetime
import time

class MbotLogger(object):
    '''
    Logger class that commands interpreted from mbot
    '''
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', '')

        # Subscribe to the topic publishing the interpretation of the ocmmand
        rospy.Subscriber('~input_interpretation',
                         ActionSlotArray,
                         self.interpretationCallback)

        rospy.Subscriber('~input_sentence',
                         String,
                         self.sentenceCallback, queue_size=1)

        # to publish the recognition
        # self.pub_sentence_recog = rospy.Publisher('~output_recognition', ActionSlotArray, queue_size=1)
        # flag to indicate that a text was received and needs to be processed
        self.interpretation_received = False
        # to store the sentece that will be received in the callback
        self.received_interpretation = None
        file_path = rospy.get_param('~log_file')
        self.logging_file = open(file_path, 'w')
        self.recognized_sentence = None
        # inform the user that the node has initialized
        rospy.loginfo("Logger is ready to receive commands")


    def sentenceCallback(self, msg):
        print('Received sentence ')
        print(msg.data)
        received_sentence = msg.data
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        self.logging_file.write('--- Sentence received: --- \n')
        self.logging_file.write('{} {} \n'.format(st, received_sentence))

    def interpretationCallback(self, msg):
        print('Received interpretation')
        output_msg_list = []
        self.logging_file.write('--- Interpretation received: --- \n')
        ts = time.time()
        st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d %H:%M:%S')
        output_msg_list.append(st)
        for action in msg.sentence_recognition:
            output_msg_list.append(action.intention)
            for slot in action.slots:
                output_msg_list.append(slot.type)
                output_msg_list.append(slot.data)
        output_msg = ' '.join(output_msg_list)
        self.logging_file.write(output_msg + ' \n')
        self.logging_file.write(' \n')
        print(output_msg)

    def wait_for_information(self):
        while not rospy.is_shutdown():
            # sleep to control the frequency of this node
            rospy.sleep(0.1)
        print('Saving log file ...')
        self.logging_file.close()

#!/usr/bin/env python

import rospy
import os
import json
from std_msgs.msg import String
from rasa_nlu.model import Interpreter


class RasaNluWrapper(object):

    def __init__(self):
        # Setup rospy and get config
        rospy.init_node("rasa_nlu_wrapper")
        self.model_name = rospy.get_param('~rasa_nlu_model', 'restaurant_sample')
        self.topic_input_text = rospy.get_param('~rasa_nlu_input_topic', '/rasa_nlu_input')
        self.topic_output = rospy.get_param('~rasa_nlu_output_topic', '/rasa_nlu_output')

        # Build paths to rasa models
        self.base_path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../../../models', self.model_name))
        self.model_path = os.path.join(self.base_path, 'generated', 'model')

    def run(self):
        # Setup interpreter
        self.interpreter = Interpreter.load(self.model_path)

        # Setup topics for incoming/outgoing messages
        self.sub = rospy.Subscriber(self.topic_input_text, String, self.process_msg)
        self.pub = rospy.Publisher(self.topic_output, String, latch=True, queue_size=1)

        # Wait until killed
        rospy.spin()

    def process_msg(self, data):
        # Run message through rasa and get returned json
        result = json.dumps(self.interpreter.parse(unicode(data.data, 'utf-8')))
        rospy.loginfo("rasa parsed message: {}".format(result))

        # Publish result
        response = String()
        response.data = result
        self.pub.publish(response)

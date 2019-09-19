#!/usr/bin/env python

import sys
import rospy
from mdr_sciroc_elevator.srv import *

def extract_intent_client(msg):
    rospy.wait_for_service("nlu_intent")
    try:
        extract_intent = rospy.ServiceProxy("nlu_intent", RasaInterpreter)
        intent = extract_intent(msg)
        return intent.intent

    except rospy.ServiceException, e:
        print("Service call failed")

if __name__== "__main__":
    intent = extract_intent_client("yes, i can")
    print("Intent ", intent)

#!/usr/bin/env python3 

from rasa.nlu.model import Interpreter 
import rospy 

def rasa_pub():
    model_path = "/home/lucy/ros/kinetic/src/mas_models/rasa_nlu_models/common/sciroc/elevator"
    interpreter = Interpreter(model_path)

if __name__ == "__main__":
    rasa_pub()
#!/usr/bin/env python3 

from rasa.nlu.model import Interpreter 
import rospy 
from mdr_sciroc_elevator.srv import rasa_interpreter 

class RasaNLU():
    def __init__(self,path):
        # Loads Rasa NLU 
        self.interpreter = Interpreter.load(path)
        
if __name__ == "__main__":
    model_path = "/home/lucy/ros/kinetic/src/mas_models/rasa_nlu_models/common/sciroc/elevator/nlu"
    rospy.init_node("nlu_model", anonymous=True)
    rasa = RasaNLU(model_path)

#!/usr/bin/env python3 
import warnings
warnings.filterwarnings("ignore", category=FutureWarning)
from rasa.nlu.model import Interpreter 
import rospy 
from mdr_sciroc_elevator.srv import RasaInterpreter, RasaInterpreterResponse 

class RasaNLU():
    def __init__(self,path):
        # Loads Rasa NLU 
        self.interpreter = Interpreter.load(path)
    
    def create_srv(self):
        self.service = rospy.Service("nlu_intent", RasaInterpreter, self.extract_intent)
        print("Ready to extract intent of the message")
    
    def extract_intent(self,msg):
        interpretation = self.interpreter.parse(msg.message)
        return RasaInterpreterResponse(interpretation['intent']['name'])
        
if __name__ == "__main__":
    model_path = "/home/lucy/ros/kinetic/src/mas_models/rasa_nlu_models/common/sciroc/elevator/nlu"
    rospy.init_node("nlu_model", anonymous=True)
    rasa = RasaNLU(model_path)
    rasa.create_srv()
    rospy.spin()

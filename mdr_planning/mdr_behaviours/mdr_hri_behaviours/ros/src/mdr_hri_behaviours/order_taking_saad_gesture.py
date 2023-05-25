"""
Heartmet Challenge -2023
Authors: Zain Ul Haq, Khawaja Saad
zainey4@gmail.com
"""
import rospy
import actionlib
from sensor_msgs.msg import Image
from std_msgs.msg import String
import random
from mdr_listen_action.msg import ListenAction, ListenGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_detect_gesture.msg import DetectGestureAction, DetectGestureGoal, DetectGestureFeedback, DetectGestureResult

class OrderTaking(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'order_taking',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['destination_locations'],
                                   input_keys=['command'])

        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)

        # Load spacy NER model
        
        self.objects_list = {"1":"Pringles","2":"Soup Can","3":"Windex Bottle","4":"T-shirt","5":"Spatula"}
        self.loc_list = {"1":"living_room_shelf","2":"dining_table_far_view","3":"living_room","4":"dining_table","5":"kitchen"}
        # Define a list of possible friendly responses from the robot
        self.friendly_responses = ["Sure, I can do that.", "No problem.", "Okay, I'm on it.", "Consider it done."]
        self.gesture_try=0
        # Define a list of possible error responses from the robot
        self.error_responses = ["I'm sorry, I didn't understand that. Please try again.", "I didn't quite catch that. Can you repeat it?", "Sorry, I'm having trouble understanding your gesture."]
        self.client = actionlib.SimpleActionClient('mdr_actions/detect_gesture_server', DetectGestureAction)
        self.client.wait_for_server()
        self.goal = DetectGestureGoal()
        self.pub_obj = rospy.Publisher('heartmet/target_object', String, queue_size=1)
        self.object_map = {"windex_bottle" : ["windex bottle","windex", "cleaner", "sprayer", "bottle of windex"],
                           "pringles" : ["pringles can", "pringles", "prings"],
                           "soup" : ["campbell soup","soup", "can of soup", "soup can"],
                           "shirt" : ["t-shirt", "shirt", "t shirt", "black t-shirt", "black shirt", "black t shirt"],
                           "spatula" : ["spatula", "spoon", "ladel"]}
        self.location_map = {"living_room" : ["living room", "livingroom"], 
                             "dining_table" : ["dining table", "dining room", "table", "diningtable", "diningroom", "dining"],
                             "kitchen" : ["kitchen"],
                             "bathroom" : ["bathroom","restroom", "bath room", "rest room"],
                             "bedroom": ["bedroom", "bed room"]}
            
        # Define a function to generate a friendly response from the robot
    def generate_friendly_response(self):
        return random.choice(self.friendly_responses)

    # Define a function to generate an error response from the robot
    def generate_error_response(self):
        return random.choice(self.error_responses)

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        self.say(text)
        
    

    def process_confirmation(self):
        while True:
            # Listen for user confirmation
            confirmation = self.listen_to_audio()
            
            # Process user confirmation
            if confirmation is not None:
                if "yes" in confirmation.lower():
                    return True
                elif "no" in confirmation.lower():
                    return False
                else:
                    print("Sorry, I didn't understand. Please say yes or no.")
                    return None
            else:
                print("Sorry, I didn't catch that. Can you please repeat?")
                return None
    
  
    def list_items(self,item_dict):
        # self.say("I will be looking for object via hand gestures")
        # rospy.sleep(1)
        if self.gesture_try==0:
            self.say_this("Please chose your gesture according to following objects list")
            rospy.sleep(3)
            for enteries in list(item_dict.items()):
                if enteries[0]==0:
                    self.say("For "+enteries[1]+" show me "+ enteries[0] + " finger")
                else:
                    self.say("For "+enteries[1]+" show me "+ enteries[0] + " fingers")
                    rospy.sleep(1)
            
        # rospy.sleep(2)

        self.gesture_call(state = True)
        if self.gesture_result is not None:
            finger = self.gesture_result.gesture_selection
            if finger =="-1":
                item = None
            else:
                if finger in list(item_dict.keys()):
                    item = set([item_dict[finger]])
                else:
                    item = None
            return item

    def gesture_call(self, state):
        self.goal.start = state
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        print(self.client.get_result())
        self.gesture_result = self.client.get_result()
    
    def gesture_confirmation(self):
        self.say_this("Could you show thumbs up to confirm or thumbs down to decline")
        rospy.sleep(3)
        self.gesture_call(state = True)
        gest = self.gesture_result.gesture
        if gest=='Thumbs up':
            return True
        elif gest=='Thumbs down':
            return False
        else:
            return None

    def get_confirmation(self):
        confirmed = None
        statement_retries = 2
        if confirmed==None:
            self.say_this("I will be looking for confirmation via hand gestures")
            rospy.sleep(2)
            for tryi in range(statement_retries):
                confirmed = self.gesture_confirmation()
                if confirmed==None:
                    self.say_this(self.generate_error_response())
                else:
                    return confirmed
        if confirmed==None:
            self.say_this("Sorry I could not confirm your response.")
        return confirmed

    def get_gesture_info(self):
        objects=None
        locations=None
        gesture_retries = 2
        if not objects or not locations:
            self.say_this("I will be looking for your gestures input now")
            if not objects:
                for tryi in range(gesture_retries):
                    if tryi>0:
                        self.say_this(self.generate_error_response())
                    
                    # self.say_this("Could you please specify which object you'd like me to fetch")
                    #rospy.sleep(3)
                    objects = self.list_items(self.objects_list)
                    self.gesture_try=1
                    if objects:
                        break
                if not objects:
                    self.say_this("I'm sorry, but I didn't get any object to fetch. Feel free to call me if you want something.")
            if not locations and objects:
                for tryi in range(gesture_retries):
                    if tryi<0:
                        self.say_this(self.generate_error_response())
                    self.say_this(f"Could you please specify where you'd like me to fetch the {', '.join(objects)} from?")
                    #rospy.sleep(3)
                    locations = self.list_items(self.loc_list)
                    if locations:
                        break
                if not locations:
                    self.say_this("I'm sorry, but I couldn't register any gesture for a location to fetch from. I am afraid I will have to abort the task now")
                    self.locations = None
        return objects, locations

    def execute(self, userdata):
        rospy.loginfo("Interacting with human through gestures...")

        self.say_this("What can I fetch for you?")
        repeated_questions = 0
        ongoing_conversation = True

        while ongoing_conversation and repeated_questions < self.number_of_retries:
            objects = None
            locations = None
            repeated_statements = 0
            statement_retries = 2

            # if objects is None or locations is None:
            objects, locations = self.get_gesture_info()
            if objects is not None and locations is not None:
                response = f"Just to confirm, You want me to bring {', '.join(objects)} from {', '.join(locations)}. Is that correct?"
                self.say_this(response)
                
            # self.confirm(objects, locations)
 

                confirmed = False
                while not confirmed:
                    confirmed_items = self.get_confirmation()
                    
                
                    if confirmed_items:
                        confirmed = True
                        ongoing_conversation = False
                        self.succeeded = True
                        userdata.destination_locations=[locations]
                        self.say_this(f"Fetching {', '.join(objects)} from {', '.join(locations)}.")
                        
                    else:
                        self.say_this("I'm sorry, I might have not recognized the object or location correctly. Can you please repeat?")
                        confirmed = True
                        repeated_questions += 1
            else:
                repeated_questions += 1
                    

        if not ongoing_conversation:
            return 'succeeded'

        return 'failed'


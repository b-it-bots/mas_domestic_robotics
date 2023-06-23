#! /usr/bin/env python

import rospy
import actionlib
from sensor_msgs.msg import Image
from std_msgs.msg import String
import random
import speech_recognition as sr
import time
from collections import deque
from mdr_listen_action.msg import ListenAction, ListenGoal
import spacy

from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_detect_gesture.msg import DetectGestureAction, DetectGestureGoal, DetectGestureFeedback, DetectGestureResult


class InteractionClient(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'InteractionClient',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['destination_locations'],
                                   input_keys=['command'])
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        # self.number_of_retries = 3
        self.timeout = kwargs.get('timeout', 120.)

        self.threshold = kwargs.get('threshold', 0.68)
        self.current = rospy.Time.now()

        # Load spacy NER model
        
        model_directory ='/home/lucy/ros/noetic/src/mas_models/model-latest'
        self.nlp_ner = spacy.load(model_directory)

        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.mic = sr.Microphone(device_index=9)
        # Define a list of possible friendly responses from the robot
        self.friendly_responses = ["Sure, I can do that.", "No problem.", "Okay, I'm on it.", "Consider it done."]

        # Define a list of possible error responses from the robot
        self.error_responses = ["I'm sorry, I didn't understand that. Please try again.", "I didn't quite catch that. Can you repeat it?", "Sorry, I'm having trouble understanding you. Please speak more clearly."]
        self.r.pause_threshold = 1.5  # Adjust the value as needed
        self.objects_list = {"1":"Pringles","2":"Soup Can","3":"Windex Bottle","4":"T-shirt","5":"Spatula"}
        self.loc_list = {"1":"living_room","2":"hall","3":"reading_room","4":"dining"}
        self.props = {"1":"object","2":"location","3":"both"}
        self.client = actionlib.SimpleActionClient('mdr_actions/detect_gesture_server', DetectGestureAction)
        self.client.wait_for_server()
        self.goal = DetectGestureGoal()
        self.interface_flag=0   # 0 for speech and  1 for gesture   
        self.pub_obj = rospy.Publisher('heartmet/target_object', String, queue_size=1)
        self.object_map = {"windex_bottle" : ["windex bottle","windex", "cleaner", "sprayer", "bottle of windex"],
                           "pringles" : ["pringles can", "pringles", "prings", "pringle", "pringle can"],
                           "soup" : ["campbell soup","soup", "can of soup", "soup can", "tomato soup", "campbell"],
                           "shirt" : ["t-shirt", "shirt", "t shirt", "black t-shirt", "black shirt", "black t shirt", "tshirt"],
                           "spatula" : ["spatula", "spoon", "ladel"]}
        self.location_map = {"shelf" : ["living room", "livingroom", "living"], 
                             "dining" : ["dining table", "dining room", "table", "diningtable", "diningroom", "dining"],
                             "kitchen" : ["kitchen"],
                             "bathroom" : ["bathroom","restroom", "bath room", "rest room"],
                             "bedroom": ["bedroom", "bed room"],
                             "hall": ["hall"],
                             "reading_room" : ["readingroom","reading room", "reading"]}
    
    def gesture_call(self, state):
        self.goal.start = state
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        print(self.client.get_result())
        self.gesture_result = self.client.get_result()

    def list_items(self,item_dict):
        self.say("I will be looking for locations via hand gestures")
        rospy.sleep(1)
        for locs in list(item_dict.items()):
            self.say("For "+locs[1]+" show me "+ locs[0])
            rospy.sleep(1)
        self.say("fingers respectively")
        rospy.sleep(3)
        self.gesture_call(state = True)
        finger = self.gesture_result.gesture_selection
        if finger =="-1":
            item = None
        else:
            if finger in list(item_dict.keys()):
                item = set([item_dict[finger]])
            else:
                item = None
        return item

    def gesture_confirmation(self):
        self.say_this("Could you show thumbs up to confirm or thumbs down to decline", time_out=3)
        rospy.sleep(3)
        self.gesture_call(state = True)
        gest = self.gesture_result.gesture
        if gest=='Thumbs up':
            return True
        elif gest=='Thumbs down':
            return False
        else:
            return None

    def generate_friendly_response(self):
        return random.choice(self.friendly_responses)

    def generate_error_response(self):
        return random.choice(self.error_responses)
            
    def listen_to_audio(self):
        try:
            with self.mic as source:
                self.r.adjust_for_ambient_noise(source)
                print("Say something!")
                audio = self.r.listen(source)
                #audio = self.r.record(source,duration=5)
                try:
                    user_input = self.r.recognize_google(audio)
                    user_input = user_input.lower()
                    print("You said:", user_input)
                    return user_input
                except:
                    user_input = self.r.recognize_sphinx(audio)
                    user_input = user_input.lower()
                    print("You said:", user_input)
                    return user_input
        except sr.UnknownValueError:
            print(self.generate_error_response())
            return None
        except sr.RequestError as e:
            print("Sorry, there was an error processing your request. Please try again later.")
            return None
            
    def speech_execution(self):
        print('speech execution')
        cur = int(rospy.Time.now().to_sec())
        # print("cur",cur)
        cur1 = int(self.current.to_sec())
        # print("cur1",cur1)
        if cur - cur1 >= 10:
            return False
        else:
            return True

    def detect_word(self,sentence,word_map):
        for i in list(word_map.items()):
            #print(i)
            for word in i[1]:
                if sentence:
                    if word.lower() in sentence:
                        return set([i[0]])
        return None

    def get_map(self,objects,locations):
        if locations:
            loc = list(locations)[0]
            loc = self.detect_word(loc,self.location_map)
        else:
            loc = None
        if objects:
            obj = list(objects)[0]
            obj = self.detect_word(obj,self.object_map)
        else:
            obj = None
        return obj, loc
            
    def process_user_input(self):
        user_input = self.listen_to_audio()
        try:
            entities = self.nlp_ner(user_input)
            objects = []
            locations = []
            for ent in entities.ents:
                if ent.label_ == 'OBJ':
                    objects.append(str(ent.text))
                    print(str(ent.text))
                elif ent.label_ == 'LOC':
                    #loc = str(ent.text)
                    locations.append(str(ent.text))
                    print(str(ent.text))
                    '''print(loc)
                    try:
                        loc = str(ent.text)
                        #loc_l = loc.split()
                        #print(loc_l)
                        #loc1 = "_".join(loc_l)
                        #print(loc1)
                    except Exception as e:
                        print(e)
                        loc1 = ent.text'''
                    #locations.append(loc)
            objects, locations = set(objects), set(locations)
            if not objects and user_input:
                objects = self.detect_word(user_input,self.object_map)
            if not locations and user_input:
                locations = self.detect_word(user_input,self.location_map)
            objects, locations = self.get_map(objects,locations)
            '''if not objects and user_input:
                objects = self.detect_word(user_input,self.object_map)
            if not locations and user_input:
                locations = self.detect_word(user_input,self.location_map)
            objects, locations = self.get_map(objects,locations)'''
            #return set(objects), set(locations)
            #print("outs:",objects, locations)
            return objects, locations
        except Exception as e:
            print(e)
            print("error")
            return None, None
    
    def audio_confirmation(self):
        # Listen for user confirmation
        user_input = self.listen_to_audio()
        # Process user confirmation
        if user_input is not None:
            if "yes" in user_input.lower():
                return True
            elif "no" in user_input.lower():
                return False
            else:
                self.say_this("Sorry, I didn't understand. Please say yes or no.")
                return None
        else:
            self.say_this("Sorry, I didn't catch that. Can you please repeat?")
            return None
        
    def audio_prop(self):
        # Listen for user confirmation
        user_input = self.listen_to_audio()
        # Process user confirmation
        if user_input is not None:
            if "item" in user_input.lower():
                return "item"
            elif "location" in user_input.lower():
                return "location"
            elif "both" in user_input.lower():
                return "both"
            else:
                self.say_this("Sorry, I didn't understand. Please say yes or no.")
                return None
        else:
            self.say_this("Sorry, I didn't catch that. Can you please repeat?")
            return None
            
    def say_this(self, text, time_out=3):
        rospy.loginfo('Saying: %s' % text)
        self.say(text)
        rospy.sleep(time_out)

    def confirm_loop(self):
        statement_retries = 2

        if self.interface_flag == 0:
            for _ in range(statement_retries):
                confirmed = self.audio_confirmation()
                self.interface_flag=1
                if confirmed is not None:
                    self.interface_flag=0   # 0 for speech and  1 for gesture   
                    return confirmed

        self.say_this("I will be looking for confirmation via hand gestures")

        if self.interface_flag == 1:
            for _ in range(statement_retries):
                confirmed = self.gesture_confirmation()
                if confirmed is None:
                    # self.interface_flag=0
                    self.say_this(self.generate_error_response())
                else:
                    self.interface_flag=1
                    return confirmed

        self.say_this("Sorry I could not confirm your response.")
        return None

    def gesture_confirm_loop(self):
        statement_retries = 2

        self.say_this("I will be looking for confirmation via hand gestures")

        for _ in range(statement_retries):
            confirmed = self.gesture_confirmation()
            if confirmed is None:
                self.say_this(self.generate_error_response())
            else:
                return confirmed

        self.say_this("Sorry I could not confirm your response.")
        return None
    
    def get_info(self, objects=None, locations=None):
        audio_retries = 2
        if self.interface_flag==0:
            #self.say_this("What can I fetch for you and from which location?")
            #objects, locations = self.process_user_input()
            #print(objects, locations)
            for tryi in range(audio_retries):
                if not objects and not locations:
                    if tryi>=audio_retries-1:
                        self.say_this(self.generate_error_response())
                    self.say_this("Could you tell me both object and also its location if you can")
                    objects, locations = self.process_user_input()
                elif not objects and locations:
                    self.say_this(self.generate_error_response())
                    self.say_this("Could you repeat the object")
                    objects, _ = self.process_user_input()
                elif objects and not locations:
                    self.say_this(self.generate_error_response())
                    self.say_this("Could you repeat the location")
                    _, locations = self.process_user_input()
                else:
                    break
        else:
            gesture_retries = 2
            if not objects or not locations:
                self.say_this("I will be looking for your gestures input")
                if not objects:
                    for tryi in range(gesture_retries):
                        if tryi<0:
                            self.say_this(self.generate_error_response())
                        self.say_this("Could you please specify which object you'd like me to fetch")
                        #rospy.sleep(3)
                        objects = self.list_items(self.objects_list)
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
                        self.say_this("I'm sorry, but I didn't get any location to fetch from. I try to find the object from the environment.")
                        locations = set(["any_location"])
        return objects, locations
        
    def get_props(self, prop=None):
        audio_retries = 2
        self.say_this("Can you tell me which is wrong, object, location or both")
        for tryi in range(audio_retries):
            prop = self.audio_prop()
            if prop:
                return prop
        gesture_retries = 2
        if not prop:
            for tryi in range(gesture_retries):
                self.say_this("I'm sorry, but I could't properly catch what you might have said. Can you please provide the gesture for it?")
                #rospy.sleep(6)
                prop = self.list_items(self.props)
                if prop:
                    break
            if not prop:
                prop = "both"
        return prop
        
    def execute(self, userdata):
        rospy.loginfo('Interacting through speech and gestures ...')
        self.say_this("Can I fetch something for you?")
        
        if not self.comfirm_loop():
            self.say_this("If you change your mind, I will be right here to serve you.")
            return 'failed'
        
        retries = 2
        objects, locations = None, None
        
        for _ in range(retries):
            objects, locations, confirmed = self.confirm_and_get_info(objects, locations)
            
            if not confirmed:
                objects, locations = self.handle_not_confirmed()
            else:
                return self.handle_success(objects, locations, userdata)

        self.say_this("Sorry I could not identify the object and or location. Please try again later",time_out=2)
        return 'failed'

    def confirm_and_get_info(self, objects, locations):
        
        objects, locations = self.get_info(objects, locations)
      
        self.say_this(f"Just to confirm, You want me to bring {', '.join(objects)} from {', '.join(locations)}. Is that correct?")
        confirmed = self.comfirm_loop()
        return objects, locations, confirmed

    def handle_not_confirmed(self):
        prop = self.get_props()
        if prop == "object":
            return None, locations
        elif prop == "location":
            return objects, None
        else:
            return None, None

    def handle_success(self, objects, locations, userdata):
        loc = list(locations)[0]
        obj = list(objects)[0]
        userdata.destination_locations = [loc] 
        self.pub_obj.publish(obj)
        self.say_this(f"I will fetch {', '.join(objects)} from {', '.join(locations)}.")
        return 'succeeded'

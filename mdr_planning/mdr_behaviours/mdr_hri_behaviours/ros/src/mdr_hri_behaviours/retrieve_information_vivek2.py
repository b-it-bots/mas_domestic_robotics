#! /usr/bin/env python3

import rospy
import actionlib
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import random
import speech_recognition as sr
import requests
import time
from collections import deque
from mdr_listen_action.msg import ListenAction, ListenGoal
import spacy
#import numpy as np
#import threading
import re
#from cv_bridge import CvBridge, CvBridgeError
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_detect_gesture.msg import DetectGestureAction, DetectGestureGoal, DetectGestureFeedback, DetectGestureResult


class InteractionClient(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'InteractionClient',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['destination_locations', 'object_tilt'],
                                   input_keys=['command'])
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        # self.number_of_retries = 3
        self.timeout = kwargs.get('timeout', 120.)
        self.threshold = kwargs.get('threshold', 0.68)
        self.current = rospy.Time.now()
        #self.image_pub = rospy.Publisher("image_topic_2", Image, queue_size=10)
        # Load spacy NER model
        model_directory = '/home/lucy/ros/noetic/src/mas_models/model-latest'
        self.nlp_ner = spacy.load(model_directory)
        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        #self.r.dynamic_energy_threshold = False
        #self.r.energy_threshold = 2000
        self.mic = sr.Microphone()#device_index=9)#device_index=9) for razor microphone 
        with self.mic as source:
            self.r.adjust_for_ambient_noise(source)
        #self.disp_imager = np.ones((480,640,3),dtype=np.uint8)
        self.stop_image = False
        # Define a list of possible friendly responses from the robot
        self.friendly_responses = ["Sure, I can do that.", "No problem.", "Okay, I'm on it.", "Consider it done."]
        # Define a list of possible error responses from the robot
        self.error_responses = ["I'm sorry, I didn't understand that. Please try again.",
                                "I didn't quite catch that. Can you repeat it?", "Sorry, I'm having trouble understanding you. Please speak more clearly."]
        #self.r.pause_threshold = 1.0  # Adjust the value as needed
        self.objects_list = {"1": "pringles", "2": "spatula", "3": "soup", "4": "windex", "5": "tshirt"}
        self.loc_list = {"1": "living_room", "2": "hall","3": "reading_room", "4": "dining"}
        self.props = {"1": "object", "2": "location", "3": "both"}
        self.client = actionlib.SimpleActionClient('mdr_actions/detect_gesture_server', DetectGestureAction)
        #self.bridge = CvBridge()
        self.client.wait_for_server()
        self.goal = DetectGestureGoal()
        self.pub_obj = rospy.Publisher('heartmet/target_object', String, queue_size=1)
        self.object_map = {"windex": ["windex bottle", "windex", "cleaner", "sprayer", "bottle of windex"],
                           "pringles": ["pringles can", "pringles", "prings", "pringle", "pringle can", "chips", "chips can"],
                           "soup": ["campbell soup", "soup", "can of soup", "soup can", "tomato soup", "campbell"],
                           "tshirt": ["t-shirt", "shirt", "t shirt", "black t-shirt", "black shirt", "black t shirt", "tshirt"],
                           "spatula": ["spatula", "spoon", "ladel"]}
        
        # Use this only when using heartmet_arena map
        # self.location_map = {"shelf_close_left": ["living room", "livingroom", "living"],
        #                      "dining": ["dining table", "dining room", "table", "diningtable", "diningroom", "dining"],
        #                      "kitchen": ["kitchen"],
        #                      "bathroom": ["bathroom", "restroom", "bath room", "rest room"],
        #                      "bedroom": ["bedroom", "bed room"],
        #                      "hall": ["hall"],
        #                      "reading_room": ["readingroom", "reading room", "reading"]}

        # Use this only when using brsu-c069 map
        self.q=0
        self.location_map = {"living_room": ["living room", "livingroom", "living","reading","reading room"],
                             "dining_table": ["dining table", "dining room", "table", "diningtable", "diningroom", "dining"],
                             "kitchen_counter": ["kitchen", "kitchen counter", "counter"],
                             "living_room_shelf": ["living room shelf", "shelf"],
                             "sofa": ["sofa", "living room sofa"],
                             "hall_shelf": ["hall", "hall cabinet","hall shelf"],
                             "living_room_cabinet": ["living room cabinet", "living cabinet", "livingroomcabinet", "cabinet"]}
    
    def publish_image(self,img_path):
        print(1)
        #disp_imager1 = cv2.imread(img_path)
        #image_message = self.bridge.cv2_to_imgmsg(disp_imager1, encoding="passthrough")
        #self.image_pub.publish(image_message)
    
    def display_image(self):
        while 1:
            #image = cv2.imread(image_path)
            cv2.namedWindow("Image", cv2.WINDOW_NORMAL)
            cv2.setWindowProperty("Image", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            cv2.imshow('Image', self.disp_imager)
            cv2.waitKey(1)  # Display the image for 10 seconds (10,000 milliseconds)
            if self.stop_image:
                cv2.destroyAllWindows()
                break
            #self.q=self.q+1
        cv2.destroyAllWindows()

    def gesture_call(self, state):
        self.goal.start = state
        self.client.send_goal(self.goal)
        self.client.wait_for_result()
        print(self.client.get_result())
        self.gesture_result = self.client.get_result()

    def list_items(self, item_dict, ):
        # self.say("I will be looking for object via hand gestures")
        # rospy.sleep(1)
        self.say("Please show me the gesture corresponding to the")
        for locs in list(item_dict.items()):
            if locs[0]==0:
                self.say("For "+locs[1]+" put up "+ locs[0] + " finger")
            else:
                self.say("For "+locs[1]+" put up "+ locs[0] + " fingers")
            rospy.sleep(1)
        #self.say("fingers respectively")
        #rospy.sleep(3)
        self.gesture_call(state=True)
        finger = self.gesture_result.gesture_selection
        if finger == "-1":
            item = None
        else:
            if finger in list(item_dict.keys()):
                item = set([item_dict[finger]])
            else:
                item = None
        return item

    def list_items2(self, item_dict, item):
        # self.say("I will be looking for object via hand gestures")
        # rospy.sleep(1)
        self.say("Please show me the gesture corresponding to the "+item+" as shown on the display")
        '''for locs in list(item_dict.items()):
            if locs[0]==0:
                self.say("For "+locs[1]+" put up "+ locs[0] + " finger")
            else:
                self.say("For "+locs[1]+" put up "+ locs[0] + " fingers")
            rospy.sleep(1)'''
        #self.say("fingers respectively")
        #rospy.sleep(3)
        self.gesture_call(state=True)
        finger = self.gesture_result.gesture_selection
        if finger == "-1":
            item = None
        else:
            if finger in list(item_dict.keys()):
                item = set([item_dict[finger]])
            else:
                item = None
        return item

    def gesture_confirmation(self):
        self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide4.PNG")
        #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide4.PNG")
        self.say_this("Could you show thumbs up to confirm or thumbs down to decline", time_out=3)
        #rospy.sleep(3)
        self.gesture_call(state=True)
        gest = self.gesture_result.gesture
        if gest == 'Thumbs up':
            return True
        elif gest == 'Thumbs down':
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
                print("Say something!")
                #audio = self.r.listen(source,timeout=8)
                audio = self.r.record(source,duration=7)
                rospy.loginfo('✅--------------heard the person going to recognize------------------✅')
                try:
                    #user_input = self.r.recognize_google(audio,Language = 'en-in') #'en-us'
                    user_input = self.r.recognize_whisper(audio, language="english")#, model="tiny")
                    user_input = user_input.lower()
                    user_input = re.sub(r'[^\w]', ' ', user_input)
                    print("You said:", user_input)
                    return user_input
                except:
                    user_input = self.r.recognize_sphinx(audio)
                    user_input = user_input.lower()
                    print("You said:", user_input)
                    return user_input
        except sr.UnknownValueError:
            self.say_this("Sorry, there was an error processing your request. Please try again later.")
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

    def detect_word(self, sentence, word_map):
        for i in list(word_map.items()):
            # print(i)audio
            for word in i[1]:
                if sentence:
                    if word.lower() in sentence:
                        return set([i[0]])
        return None

    def get_map(self, objects, locations):
        if locations:
            loc = list(locations)[0]
            loc = self.detect_word(loc, self.location_map)
        else:
            loc = None
        if objects:
            obj = list(objects)[0]
            obj = self.detect_word(obj, self.object_map)
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
                    # loc = str(ent.text)
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
                    # locations.append(loc)
            objects, locations = set(objects), set(locations)
            if not objects and user_input:
                objects = self.detect_word(user_input, self.object_map)
            if not locations and user_input:
                locations = self.detect_word(user_input, self.location_map)
            objects, locations = self.get_map(objects, locations)
            '''if not objects and user_input:
                objects = self.detect_word(user_input,self.object_map)
            if not locations and user_input:
                locations = self.detect_word(user_input,self.location_map)
            objects, locations = self.get_map(objects,locations)'''
            # return set(objects), set(locations)
            # print("outs:",objects, locations)
            return objects, locations
        except Exception as e:
            print(e)
            print("error")
            return None, None

    def audio_confirmation(self):
        # Listen for user confirmation
        self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide7.PNG")
        #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide7.PNG")
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
        self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide5.PNG")
        #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide5.PNG")
        self.say_this("Can you tell me which is wrong, object, location or both", time_out=3)
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
                self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
                #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
                self.say_this("Sorry, I didn't understand. Please say object, location or both")
                return None
        else:
            self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
            #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
            self.say_this("Sorry, I didn't catch that. Can you please repeat?")
            return None

    def say_this(self, text, time_out=2):
        rospy.loginfo('Saying: %s' % text)
        self.say(text)
        rospy.sleep(time_out)

    def comfirm_loop(self):
        confirmed = None
        statement_retries = 2
        for tryi in range(statement_retries):
            confirmed = self.audio_confirmation()
            if confirmed == None:
                pass
            else:
                return confirmed
        if confirmed == None:
            self.say_this("I will be looking for confirmation via hand gestures", time_out=3)
            for tryi in range(statement_retries):
                confirmed = self.gesture_confirmation()
                if confirmed == None:
                    self.say_this(self.generate_error_response())
                else:
                    return confirmed
        if confirmed == None:
            self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
            #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
            self.say_this("Sorry I could not confirm your response.")
        return confirmed

    def get_info(self, objects=None, locations=None):
        user_utterances = 10 # max times the user can say before moving on to gesture 
        self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide2.PNG")
        RASA_SERVER_URL = "http://192.168.50.22:5005/webhooks/rest/webhook"

        # obtain audio from the microphone
        r = sr.Recognizer()
        with sr.Microphone() as source:
            r.adjust_for_ambient_noise(source)
            r.dynamic_energy_threshold = False
        
        response = requests.post(RASA_SERVER_URL, json={"message": "scenario 1"})
        init_message = response.json()[0]["custom"]['data']['response']

        self.say_this(init_message, time_out=3)
        while user_utterances != 0:
            with sr.Microphone() as source:
                audio = r.listen(source)

            # recognize speech using whisper
            try:
                whisper_text = r.recognize_whisper(audio, language="english", model="base")
                print("Whisper base model thinks you said: " + whisper_text)

                # Send whisper_text to Rasa server
                response = requests.post(RASA_SERVER_URL, json={"message": whisper_text})
                try:
                    data = response.json()[0]["custom"]
                    if data["text"] == "conversation_ongoing":
                        print(f"Rasa server response: {data['data']['response']}")
                        self.say_this(data['data']['response'], time_out=3)
                        user_utterances -= 1
                    elif data["text"] == "conversation_session_end":
                        try:
                            print(f"Getting {data['data']['item']} from {data['data']['location']}")
                            objects = data['data']['item']
                            locations = data['data']['location']
                            objects = set(objects.lower())
                            locations = set(locations.lower())
                        except KeyError:
                            print("KeyError. Invalid response from Rasa server.")
                        break
                except:
                    print("Invalid response from Rasa server.")
                    continue

            except sr.UnknownValueError:
                print("Whisper could not understand audio")
            except sr.RequestError as e:
                print("Could not request results from Whisper")
        gesture_retries = 2
        if not objects or not locations:
            self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
            #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
            if not objects and not locations:
                self.say_this("Sorry, I didn't catch the object and location that you might have said. I will be looking for gestures",time_out=5)
            elif not objects and locations:
                self.say_this("Sorry, I didn't catch the object you might have said. I will be looking for gestures",time_out=5)
            elif objects and not locations:
                self.say_this("Sorry, I didn't catch the location you might have said. I will be looking for gestures",time_out=5)
            if not objects:
                for tryi in range(gesture_retries):
                    self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide1.PNG")
                    #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide1.PNG")
                    #if tryi > 0:
                        #self.say_this(self.generate_error_response())
                    self.say_this("Could you show me which object you'd like me to fetch", time_out=3)
                    # rospy.sleep(3)
                    objects = self.list_items2(self.objects_list,"object")
                    if objects:
                        break
                if not objects:
                    self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
                    #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
                    self.say_this("I'm sorry, but I didn't get any object to fetch. Feel free to call me if you want something.", time_out=4)
            
            if not locations and objects:
                for tryi in range(gesture_retries):
                    self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide3.PNG")
                    #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide3.PNG")
                    if tryi > 0:
                        self.say_this(self.generate_error_response())
                    self.say_this(f"Could you please specify where you'd like me to fetch the {', '.join(objects)} from?", time_out=3)
                    # rospy.sleep(3)
                    locations = self.list_items2(self.loc_list, "location")
                    if locations:
                        break
                if not locations:
                    self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
                    #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide8.PNG")
                    self.say_this("I'm sorry, but I didn't get any location to fetch from.", time_out=3)
                    # self.locations = set(["any_location"])
                    self.locations = None
        return objects, locations

    def get_props(self, prop=None):
        audio_retries = 2        
        for tryi in range(audio_retries):
            prop = self.audio_prop()
            if prop:
                return prop
        gesture_retries = 2
        if not prop:
            for tryi in range(gesture_retries):
                self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide6.PNG")
                #self.disp_imager = cv2.imread("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide6.PNG")
                self.say_this("I'm sorry, but I could't properly catch what you might have said. Can you please provide the gesture for it?", time_out=4)
                # rospy.sleep(6)
                prop = self.list_items(self.props)
                if prop:
                    break
            if not prop:
                prop = "both"
        return prop

    def execute(self, userdata):
        rospy.loginfo('Interacting through speech and gestures ...')
        #self.say_this("Can I fetch somthing for you?")
        #confirmed = self.comfirm_loop()
        '''if not confirmed:
            # self.say_this("If you change your mind, I will right here to serve you.", time_out=3)
            self.say_this("I will fetch now", time_out=2)
            # userdata.destination_locations = ['dining'] #['dining_table']
            # self.pub_obj.publish('pringles')
            return 'failed'''
        #else:
        self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide9.PNG")
        self.say_this("Hello, I am Lucy, here to serve you. What can I fetch for you", time_out=4)
        #image_thread = threading.Thread(target=self.display_image,daemon=True)
        # Start the thread
        #image_thread.start()
        retries = 2
        objects, locations = None, None
        for i in range(retries):
            objects, locations = self.get_info(objects, locations)
            if objects and locations:
                self.say_this(f"Just to confirm, You want me to bring {', '.join(objects)} from {', '.join(locations)}. Is that correct?", time_out=3)
                confirmed = self.comfirm_loop()
                if not confirmed:
                    prop = self.get_props()
                    if prop == "object":
                        objects = None
                    elif prop == "location":
                        locations = None
                    else:
                        objects, locations = None, None
                else:
                    loc = list(locations)[0]
                    obj = list(objects)[0]
                    userdata.destination_locations = [
                        loc]  # ['dining_table']
                    self.pub_obj.publish(obj)
                    # userdata.objects = [obj]
                    self.say_this(f"I will fetch {', '.join(objects)} from {', '.join(locations)}.", time_out=3)

                    if loc == "hall":
                        userdata.object_tilt = -0.1
                    elif loc == "reading_room":
                        userdata.object_tilt = -0.7
                    elif loc == "shelf_close_left":
                        userdata.object_tilt = -0.1
                    elif loc == "dining":
                        userdata.object_tilt = -0.1
                    else:
                        userdata.object_tilt = -0.1
                    '''if image_thread.is_alive():
                        self.stop_image = True
                        cv2.destroyAllWindows()  # Close the image window
                        image_thread.join()
                    cv2.destroyAllWindows()'''
                    return 'succeeded'
        if objects:
            # self.say_this("Sorry I could not identify the object and or location. Please try again later",time_out=3)
            self.say_this(f"I will fetch {', '.join(objects)}", time_out=3)
            # userdata.destination_locations = ['dining'] #['dining_table']
            # self.pub_obj.publish('pringles')
        '''if image_thread.is_alive():
            self.stop_image = True
            cv2.destroyAllWindows()  # Close the image window
            image_thread.join()
        cv2.destroyAllWindows()'''
        return 'failed'
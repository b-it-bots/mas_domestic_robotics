"""
Heartmet Challenge -2023
Authors: Zain Ul Haq, Khawaja Saad
zainey4@gmail.com
"""
import rospy
import actionlib
import random
import speech_recognition as sr
from keras.models import load_model

from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_listen_action.msg import ListenAction, ListenGoal
import spacy


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
        
        model_directory ='/home/lucy/ros/noetic/src/mas_models/model-latest'
        self.nlp_ner = spacy.load(model_directory)

        confirmation_model_dir = '/home/lucy/ros/noetic/src/mas_models/confirmation_model/confirmation_model.h5'
        confirmation_model = load_model(confirmation_model_dir)

        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        # Define a list of possible friendly responses from the robot
        self.friendly_responses = ["Sure, I can do that.", "No problem.", "Okay, I'm on it.", "Consider it done."]

        # Define a list of possible error responses from the robot
        self.error_responses = ["I'm sorry, I didn't understand that. Please try again.", "I didn't quite catch that. Can you repeat it?", "Sorry, I'm having trouble understanding you. Please speak more clearly."]
        self.r.pause_threshold = 1.5  # Adjust the value as needed
                
        # Define a function to generate a friendly response from the robot
    def generate_friendly_response(self):
        return random.choice(self.friendly_responses)

    # Define a function to generate an error response from the robot
    def generate_error_response(self):
        return random.choice(self.error_responses)

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        self.say(text)
        

    def listen(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source)
            print("Say something!")
            audio = self.r.listen(source)
        
        try:
            user_input = self.r.recognize_google(audio)
            print("You said:", user_input)
            objects, locations = self.process_user_input(user_input)
            print(objects, locations)
            return objects, locations
        except sr.UnknownValueError:
            print(self.generate_error_response())
            return None, None
        except sr.RequestError as e:
            print("Sorry, there was an error processing your request. Please try again later.")
            return None, None

    def process_user_input(self, user_input):
        entities = self.nlp_ner(user_input)
        objects = []
        locations = []

        for ent in entities.ents:
            if ent.label_ == 'OBJ':
                objects.append(ent.text)
            elif ent.label_ == 'LOC':
                locations.append(ent.text)

        return set(objects), set(locations)

  

    def confirm(self, objects, locations):
        response = f"Just to confirm, You want me to bring {', '.join(objects)} from {', '.join(locations)}. Is that correct?"
        self.say(response)

    def listen_to_audio(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source)
            print("Say something!")
            audio = self.r.listen(source)
        
        try:
            user_input = self.r.recognize_google(audio)
            print("You said:", user_input)
            return user_input
        except sr.UnknownValueError:
            print(self.generate_error_response())
            return None
        except sr.RequestError as e:
            print("Sorry, there was an error processing your request. Please try again later.")
            return None

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

    def execute(self, userdata):
        rospy.loginfo("Interacting with human through speech...")

        self.say_this("What can I fetch for you?")
        repeated_questions = 0
        ongoing_conversation = True

        while ongoing_conversation and repeated_questions < self.number_of_retries:
            objects = None
            locations = None
            repeated_statements = 0
            statement_retries = 2

            while objects is None or locations is None:
                objects, locations = self.listen()

                if not objects:
                    repeated_statements = 0
                    while not objects:
                        if repeated_statements < statement_retries:
                            self.say_this(f"I'm sorry, but I didn't catch any object in your request. Could you please specify what you'd like me to fetch?")
                            objects , _ = self.listen()
                            repeated_statements += 1
                        else:
                            self.say_this("I'm sorry, but I could't properly catch the object that you might have said. Can you please provide the gesture for it?")
                            # add gesture code here


                if not locations and objects:
                    repeated_statements = 0
                    self.say_this(f"From where do you want me to fetch the {list(objects)[0]}")
                    while not locations:
                        if repeated_statements < statement_retries:
                            _ , locations = self.listen()
                            self.say_this("I'm sorry, but I didn't catch any location in your request. Could you please specify where you'd like me to fetch the object from?")
                        else:
                            self.say_this("I'm sorry, but I could't properly catch the location that you might have said. Can you please provide the gesture for it?")
                            # add gesture code here

                        if locations:
                            break

            self.confirm(objects, locations)

            confirmed = False
            while not confirmed:
                confirmed_items = self.process_confirmation()

                if confirmed_items is None:
                    repeated_questions += 1
                    continue
                elif confirmed_items:
                    confirmed = True
                    ongoing_conversation = False
                    self.succeeded = True
                    userdata.destination_locations=[locations]
                    self.say_this(f"Fetching {', '.join(objects)} from {', '.join(locations)}.")
                    identifier = True
                # If not confirmed, ask user to confirm again
                elif not confirmed_items:
                    print("I'm sorry, I might have not recognized the object or location correctly. Can you please repeat?")
                    speak("I'm sorry, I might have not recognized the object or location correctly. Can you please repeat?")
                    confirmed = True
                    repeated_questions += 1
                else:
                    continue
                    

        if not ongoing_conversation:
            return 'succeeded'

        return 'failed'


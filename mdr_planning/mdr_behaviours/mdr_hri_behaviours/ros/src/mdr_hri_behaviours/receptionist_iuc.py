"""
Receptionist Challenge -2023
Authors: Zain Ul Haq, Khawaja Saad, Ayusee Swain
zainey4@gmail.com
"""
import rospy
import random
import spacy
import speech_recognition as sr
from polyglot.text import Text
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from std_msgs.msg import String  # ROS standard string message
from threading import Thread #import the threading module for running background task

# Load SpaCy models
nlp_drink = spacy.load("/home/lucy/rasa_ws/spacy_model/model_drink")  # Model trained to identify drinks
nlp_name = spacy.load("/home/lucy/rasa_ws/spacy_model/model_name")  # Model trained to identify names

class ReceptionistTask(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receptionist_task',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['guest_name', 'favorite_drink','personmetadata'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)
        self.person_data={'guest_names':[], 'favorite_drinks':[], 'personmetadata':[]} ## list of string, list of string, list of ndarray
        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.r.pause_threshold = 1.5  # Adjust the value as needed

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)

    def listen_and_transcribe(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source, duration=1)
            rospy.loginfo("Listening...")
            audio = self.r.listen(source)
        try:
            return self.r.recognize_google(audio)
        except (sr.UnknownValueError, sr.RequestError):
            rospy.loginfo("I am sorry, I did not catch that. Could you please repeat?")
            return None

    def extract_information_spacy(self, sentence, model):
        try:
            doc = model(sentence)
            entities = [ent.text for ent in doc.ents]
            return entities[0] if entities else None
        except Exception as e:
            print(f"Error processing sentence with SpaCy: {e}")
            return None

    def execute(self, userdata):
        rospy.loginfo("Initiating receptionist interaction...")
        rospy.sleep(2)  # Small delay before greeting
        
        # Initial greeting
        self.say_this("Hello, I am Lucy, here to welcome you. May I know your name?")
        name_response = self.listen_and_transcribe()
        rospy.loginfo(f"{name_response}")
        if name_response:
            guest_name = self.extract_information_spacy(name_response, nlp_name)
            if guest_name:
                userdata.guest_name = guest_name
            else:
                self.say_this("I could not identify your name correctly. Let's try again.")
                return 'failed'
        else:
            return 'failed'

        self.say_this("What is your favorite drink?")
        drink_response = self.listen_and_transcribe()
        if drink_response:
            favorite_drink = self.extract_information_spacy(drink_response, nlp_drink)
            if favorite_drink:
                userdata.favorite_drink = favorite_drink
            else:
                self.say_this("I could not identify your favorite drink correctly. Let's try again.")
                return 'failed'
        else:
            return 'failed'

        self.say_this(f"Welcome, {guest_name}. I have noted that your favorite drink is {favorite_drink}. Now, please follow me to the sitting area.")
        rospy.sleep(5) 
        return 'succeeded'


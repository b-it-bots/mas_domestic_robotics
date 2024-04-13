"""
Receptionist Challenge -2023
Authors: Zain Ul Haq, Khawaja Saad
zainey4@gmail.com
"""
import rospy
import random
import speech_recognition as sr
from polyglot.text import Text
from mas_execution_manager.scenario_state_base import ScenarioStateBase

# Predefined list of known drinks
known_drinks = [
    'Espresso', 'Coffee', 'Tea', 'Water', 'Latte', 'Cappuccino', 'Mocha', 'Americano', 'Macchiato',
    # Add the rest of the drinks from your list here...
    'Kir Royale', 'Cola'  # Example continuation
]

class ReceptionistTask(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receptionist_task',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['guest_name', 'favorite_drink'])

        self.timeout = kwargs.get('timeout', 120)
        self.number_of_retries = kwargs.get('number_of_retries', 3)

        # Initialize the speech recognition module
        self.r = sr.Recognizer()
        self.r.pause_threshold = 1.5  # Adjust the value as needed

    def say_this(self, text):
        rospy.loginfo('Saying: %s' % text)
        # Integrate with a ROS publisher if you want the robot to speak out the text
        self.say(text)
    def listen_and_transcribe(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source)
            rospy.loginfo("Listening...")
            audio = self.r.listen(source)

        try:
            return self.r.recognize_google(audio)
        except (sr.UnknownValueError, sr.RequestError):
            rospy.loginfo("I am sorry, I did not catch that. Could you please repeat?")
            return None

    def extract_information(self, sentence, type_info="name"):
        text = Text(sentence, hint_language_code='en')
        if type_info == "name":
            # Extracting person's name
            person_names = [entity[0] for entity in text.entities if entity.tag == 'I-PER']
            rospy.loginfo(person_names)
            # return ' '.join(person_names) if person_names else None
            if person_names:

                return person_names[0]
            else:
                return None 

        elif type_info == "drink":
            # Identifying the favorite drink from the sentence
            favorite_drink = next((drink for drink in known_drinks if drink.lower() in sentence.lower()), None)
            rospy.loginfo(favorite_drink)
            return favorite_drink

    def execute(self, userdata):
        rospy.loginfo("Initiating receptionist interaction...")

        self.say_this("Hello, I am Lucy, here to welcome you. May I know your name?")
        name_response = self.listen_and_transcribe()
        if name_response:
            guest_name = self.extract_information(name_response, type_info="name")
            rospy.loginfo(f"guest_name extracted {guest_name}")
            if guest_name:
                userdata.guest_name = guest_name
            else:
                self.say_this("I could not identify your name correctly. Lets try again.")
                return 'failed'
        else:
            return 'failed'

        self.say_this("What is your favorite drink?")
        drink_response = self.listen_and_transcribe()
        if drink_response:
            favorite_drink = self.extract_information(drink_response, type_info="drink")
            if favorite_drink:
                rospy.loginfo(f"drink extracted {favorite_drink}")
                userdata.favorite_drink = favorite_drink
            else:
                self.say_this("I could not identify your favorite drink correctly. lets try again.")
                return 'failed'
        else:
            return 'failed'

        self.say_this(f"Welcome, {guest_name}. I have noted that your favorite drink is {favorite_drink}. Now, please follow me to the sitting area.")
        return 'succeeded'

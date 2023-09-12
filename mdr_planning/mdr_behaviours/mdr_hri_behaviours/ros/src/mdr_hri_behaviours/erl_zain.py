import spacy
import rospy
import random
import speech_recognition as sr

from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_listen_action.msg import ListenAction, ListenGoal

class InteractionClient(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'order_taking',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   output_keys=['destination_locations'],
                                   input_keys=['command'])

        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)

        model_directory ='/home/lucy/ros/noetic/src/mas_models/model-latest'
        self.nlp_ner = spacy.load(model_directory)
        self.r = sr.Recognizer()
        self.r.pause_threshold = 1.5

    def listen_to_audio(self):
        with sr.Microphone() as source:
            self.r.adjust_for_ambient_noise(source)
            print("Say something!")
            audio = self.r.listen(source)

        try:
            user_input = self.r.recognize_whisper(audio, language="english")
            print("Whisper thinks you said " + user_input)
            return user_input
        except sr.UnknownValueError:
            print("Whisper could not understand audio")
        except sr.RequestError as e:
            print("Could not request results from Whisper")

    def execute(self, userdata):
        rospy.loginfo("Interacting with human through speech...")
        self.say("Hello! What can I fetch for you?")
        
        for _ in range(self.number_of_retries + 1):
            user_input = self.listen_to_audio()
            
            if not user_input:
                self.say("Sorry, I didn't understand that. Please try again.")
                continue

            entities = self.nlp_ner(user_input)
            objects = [ent.text for ent in entities.ents if ent.label_ == 'OBJ']
            locations = [ent.text for ent in entities.ents if ent.label_ == 'LOC']

            if objects and locations:
                confirmation_msg = f"Do you want me to bring {', '.join(objects)} from {', '.join(locations)}?"
                self.say(confirmation_msg)

                confirmation = self.listen_to_audio()
                if confirmation and "yes" in confirmation.lower():
                    userdata.destination_locations = '_'.join(locations[0].split())
                    self.say(f"Fetching {', '.join(objects)} from {', '.join(locations)}.")
                    return 'succeeded'
                else:
                    self.say("I didn't understand. Could you please repeat?")

        self.say("Sorry, I couldn't understand your request after several attempts. Let's try again later.")
        return 'failed'

import rospy
import speech_recognition as sr
import re
import requests
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_retrieve_info_action.msg import RetrieveInfoResult

class RetrieveInfoSM(ActionSMBase):
    def __init__(self,timeout=120., use_whisper=True,max_recover_attempts=1):
        super(RetrieveInfoSM, self).__init__("RetrieveSpeech", [], max_recover_attempts)
        self.time_out = timeout
        self.use_whisper = use_whisper

        try:
            self.recognizer = sr.Recognizer()
            self.microphone = sr.Microphone()
        except Exception as e:
            rospy.logerr(e)

    def say_this(self):
        pass

    def listen_to_audio(self):
        try:
            with self.mic as source:
                print("Say something!")
                audio = self.r.record(source,duration=10)
                rospy.loginfo('✅--------------heard the person going to recognize------------------✅')
        except Exception as error:
                rospy.logerr(error)
                self.say_this("Sorry, there was an error processing your request. Please try again later.")
                return None
        
        if self.use_whisper:
            try: 
                user_input = self.r.recognize_whisper(audio, language="english",model='base')
                print("whisper thinks you said: ", user_input)
                user_input = user_input.lower()
                user_input = re.sub(r'[^\w]', ' ', user_input)
                return user_input
            except Exception as error:
                rospy.logerr(error)
                return None
        else:
            try:
                user_input = self.r.recognize_google(audio,language = 'en-IN') #'en-GB'
                print("Google thinks you said: ", user_input)
                user_input = user_input.lower()
                user_input = re.sub(r'[^\w]', ' ', user_input)
                return user_input
            except Exception as e:
                rospy.logerr(error)
                return None

    def running(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        user_utterances = 10 # max times the user can say before moving on to gesture 
        RASA_SERVER_URL = "http://localhost:5005/webhooks/rest/webhook"
        self.result = RetrieveInfoResult()
        
        response = requests.post(RASA_SERVER_URL, json={"message": "scenario 1"})
        init_message = response.json()[0]["custom"]['data']['response']

        self.say_this(init_message, time_out=3)
        for utterances in range(user_utterances):
            self.publish_image("/home/lucy/ros/noetic/src/mas_domestic_robotics/mdr_planning/mdr_behaviours/mdr_hri_behaviours/disp_imgs/Slide2.PNG")
            recognized_text = self.listen_to_audio() # returns either the recognized text or None

            if recognized_text is not None:
                # Send recognized_text to Rasa server
                response = requests.post(RASA_SERVER_URL, json={"message": recognized_text})
                try:
                    data = response.json()[0]["custom"]
                    if data["text"] == "conversation_ongoing":
                        print(f"Rasa server response: {data['data']['response']}")
                        self.result.object = data['data']['item']
                        self.result.location = data['data']['location']
                        self.result.confirmation = data['data']['confirmation']
                        self.say_this(data['data']['response'], time_out=3)
                    elif data["text"] == "conversation_session_end":
                        print(f"Getting {data['data']['item']} from {data['data']['location']}")
                        self.result.object = data['data']['item']
                        self.result.location = data['data']['location']
                        self.result.confirmation = data['data']['confirmation']
                        break
                except:
                    print("Invalid response from Rasa server.")
                    continue
            else:
                print("it was just a waste of one utterance in the speech part")

        return FTSMTransitions.DONE


            

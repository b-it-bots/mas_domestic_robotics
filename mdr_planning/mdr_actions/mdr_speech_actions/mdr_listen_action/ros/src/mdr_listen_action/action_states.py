import rospy
import speech_recognition as sr
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_listen_action.msg import ListenResult
from mdr_speech_recognition.speech_recognizer import SpeechRecognizer

class ListenSM(ActionSMBase):
    def __init__(self, timeout=120.,
                 model_directory='',
                 use_kaldi=True,
                 max_recovery_attempts=1):
        super(ListenSM, self).__init__('Listen', [], max_recovery_attempts)
        self.model_directory = model_directory
        self.use_kaldi = use_kaldi
        self.timeout = timeout
        self.listen_client = None

        self.recognizer = sr.Recognizer()
        if self.use_kaldi:
            try:
                self.recognizer.load_kaldi_model(model_directory=self.model_directory)
            except Exception as exc:
                rospy.logerr(exc)
                self.use_kaldi = False
                rospy.logerr('Unable to load Kaldi model. Using PocketSphinx as offline speech recognition')
        self.microphone = sr.Microphone()
        self.recognized_speech = ''

    def running(self):
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        try:
            with self.microphone as source:
                rospy.loginfo("Listening...")
                audio = self.recognizer.listen(source)
                rospy.loginfo("Got a sound; recognizing...")
                if self.use_kaldi:
                    try:
                        self.recognized_speech = self.recognizer.recognize_kaldi(audio)[0]
                    except sr.UnknownValueError:
                        rospy.logerr("Input not understood.")
                        return FTSMTransitions.DONE
                    except sr.RequestError:
                        rospy.logerr("No input received")
                        return FTSMTransitions.DONE
                else:
                   if SpeechRecognizer.check_internet_connection():
                       try:
                           self.recognized_speech = self.recognizer.recognize_google(audio)
                       except sr.UnknownValueError:
                           rospy.logerr("Google: Input not understood.")
                           return FTSMTransitions.DONE
                       except sr.RequestError:
                           rospy.logerr("Google: No input received")
                           return FTSMTransitions.DONE
                   else:
                       try:
                           self.recognized_speech = self.recognizer.recognize_sphinx(audio)
                       except sr.UnknownValueError:
                           rospy.logerr("PocketSphinx: Input not understood.")
                           return FTSMTransitions.DONE
                       except sr.RequestError:
                           rospy.logerr("PocketSphinx: No input received")
                           return FTSMTransitions.DONE
        except Exception as exc:
            rospy.logerr(exc)
        if self.recognized_speech != '':
            self.result = self.set_result(True, self.recognized_speech)
        else:
            self.result = self.set_result(False, self.recognized_speech)

        return FTSMTransitions.DONE


    def set_result(self, success, message):
        result = ListenResult()
        result.success = success
        result.message = message
        return result

#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Created on 2018.10.09

@author: Patrick Nagel, Roberto Cai Wu
"""

import rospy
import smach
import speech_recognition as sr

from mdr_listen_action.msg import ListenResult, ListenFeedback
from speech_matching.speech_matching import SpeechMatching
from mdr_speech_recognition.speech_recognizer import SpeechRecognizer

class InitializeListen(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'processing'],
                             input_keys=['error_message', 'listen_feedback'],
                             output_keys=['listen_feedback', 'init_error_message'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializeListen")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback = ListenFeedback()
            userdata.listen_feedback.status_initialization = "in_progess"
            userdata.listen_feedback.status_wait_for_user_input = "pending"
            userdata.listen_feedback.status_process_input = "pending"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        initialization_successful = True
        if initialization_successful:
            rospy.loginfo("Initialization successful!")
            return 'succeeded'
        else:
            # Example for a possible error message.
            userdata.init_error_message = "Microphones could not be initialized."
            rospy.logerr("Microphones could not be initialized.")
            return 'failed'


class WaitForUserInput(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['input_received', 'input_not_understood', 'no_input_received', 'processing'],
                             input_keys=['listen_feedback', 'accoustic_input'],
                             output_keys=['listen_feedback', 'accoustic_input',
                                          'input_error_message'])
        self.feedback_given = False
        self.input_received = False


    def callback(self, data, userdata):
        # TODO: SpeechVerifier, here!
        #sv = SpeechVerifier("../../../common/config/dict", "../../../common/config/sentence_pool.txt", data.sentences)
        #userdata.accoustic_input = sv.find_best_match()

        if data != "":
            rospy.loginfo("You said: " + data)
            self.input_received = True
        else:
            self.input_received = False
        userdata.accoustic_input = data

    def execute(self, userdata):
        rospy.loginfo("Executing state WaitForUserInput")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "in_progress"
            userdata.listen_feedback.status_process_input = "pending"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        self.model_directory = userdata.model_directory
        self.use_kaldi = userdata.use_kaldi
        self.recognizer = sr.Recognizer()
        if self.use_kaldi:
            try:
                self.recognizer.load_kaldi_model(model_directory=self.model_directory)
            except:
                self.use_kaldi = False
                rospy.logerr(sys.exc_info()[0])
                rospy.logerr('Unable to load Kaldi model. Using PocketSphinx as offline speech recognition')
        self.microphone = sr.Microphone()

        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)
        try:
            with self.microphone as source:
                rospy.loginfo("Listening...")
                audio = self.recognizer.listen(source)
                rospy.loginfo("Got a sound; recognizing...")

                """
                Google over PocketSphinx: In case there is a internet connection
                use google, otherwise use pocketsphinx for speech recognition.
                """
                recognized_speech = ""
                if self.use_kaldi:
                    try:
                        recognized_speech = self.recognizer.recognize_kaldi(audio)[0]
                    except sr.UnknownValueError:
                        userdata.input_error_message = "Input not understood."
                        rospy.logerr("Input not understood.")
                        return 'input_not_understood'
                    except sr.RequestError:
                        userdata.input_error_message = "No input received."
                        rospy.logerr("No input received")
                        return 'no_input_received'
                else:
                    if SpeechRecognizer.check_internet_connection():
                        try:
                            recognized_speech = self.recognizer.recognize_google(audio)
                        except sr.UnknownValueError:
                            userdata.input_error_message = "Input not understood."
                            rospy.logerr("Input not understood.")
                            return 'input_not_understood'
                        except sr.RequestError:
                            userdata.input_error_message = "No input received."
                            rospy.logerr("No input received")
                            return 'no_input_received'
                    else:
                        try:
                            recognized_speech = self.recognizer.recognize_sphinx(audio)
                        except sr.UnknownValueError:
                            userdata.input_error_message = "Input not understood."
                            rospy.logerr("Input not understood.")
                            return 'input_not_understood'
                        except sr.RequestError:
                            userdata.input_error_message = "No input received."
                            rospy.logerr("No input received")
                            return 'no_input_received'

        except Exception as exc:
            rospy.logerr(exc)

        self.callback(recognized_speech, userdata)

        if self.input_received:
            return 'input_received'

        userdata.input_error_message = "No input received."
        rospy.logerr("No input received.")
        return 'no_input_received'


class InitializationError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],
                             input_keys=['error_message_in', 'listen_feedback'],
                             output_keys=['listen_feedback', 'listen_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InitializationError")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "aborted"
            userdata.listen_feedback.status_wait_for_user_input = "pending"
            userdata.listen_feedback.status_process_input = "pending"
            userdata.listen_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = ListenResult()
        result.success = False
        result.message = "None"
        result.message_type = "None"
        userdata.listen_result = result
        return 'error_detected'


class ProcessInput(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'input_not_understood', 'processing'],
                             input_keys=['accoustic_input', 'listen_feedback',
                                         'input_error_message'],
                             output_keys=['listen_feedback', 'listen_result',
                                          'input_error_message'])
        self.feedback_given = False
        self.feedback_updated = False

    def execute(self, userdata):
        rospy.loginfo("Executing state ProcessInput")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "completed"
            userdata.listen_feedback.status_process_input = "in_progress"
            userdata.listen_feedback.error_detected = False
            self.feedback_given = True
            return 'processing'

        sm = SpeechMatching()
        matching_result = sm.match_sentence(userdata.accoustic_input)

        result = ListenResult()
        if matching_result[1][1] != 0:
            rospy.loginfo("This is what I think you meant: %s", matching_result[1][0])
            result.success = True
        else:
            rospy.loginfo("I'm sorry, I don't know what you said. ")
            return 'input_not_understood'

        result.message = matching_result[1][0]
        result.message_type = matching_result[0]
        userdata.listen_result = result
        return 'succeeded'


class InputError(smach.State):

    def __init__(self):
        smach.State.__init__(self, outcomes=['error_detected', 'processing'],
                             input_keys=['error_message_in', 'listen_feedback'],
                             output_keys=['listen_feedback', 'listen_result'])
        self.feedback_given = False

    def execute(self, userdata):
        rospy.loginfo("Executing state InputError")

        # Give some feedback.
        while not self.feedback_given:
            userdata.listen_feedback.status_initialization = "completed"
            userdata.listen_feedback.status_wait_for_user_input = "completed"
            userdata.listen_feedback.status_process_input = "aborted"
            userdata.listen_feedback.error_detected = True
            self.feedback_given = True
            return 'processing'

        result = ListenResult()
        result.success = False
        result.message = "None"
        result.message_type = "None"
        userdata.listen_result = result
        return 'error_detected'

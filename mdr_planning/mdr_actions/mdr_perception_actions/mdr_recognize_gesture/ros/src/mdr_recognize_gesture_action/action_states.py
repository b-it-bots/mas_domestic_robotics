from importlib import import_module
import numpy as np
import cv2

import rospy
import actionlib

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_recognize_gesture_action.msg import RecognizeGestureGoal, RecognizeGestureResult


class RecognizeGestureSM(ActionSMBase):
    def __init__(self, timeout=120.0,
                 gesture_type='gesture_type'):
        super(RecognizeGestureSM, self).__init__('RecognizeGesture', [], max_recovery_attempts)

        self.timeout = timeout
        self.gesture_type = gesture_type

    def init(self):
        return FTSMTransitions.INITIALISED

    def running(self):
        rospy.loginfo('[recognize_gesture] Recognizing gesture')

        return FTSMTransitions.DONE

    def recovering(self):
        ## TODO: implement any recovery behaviours here
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = HandleOpenResult()
        result.success = success
        return result

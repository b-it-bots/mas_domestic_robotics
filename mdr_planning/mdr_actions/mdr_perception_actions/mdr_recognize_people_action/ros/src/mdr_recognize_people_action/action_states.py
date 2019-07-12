#!/usr/bin/python
import rospy

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_recognize_people_action.msg import RecognizePeopleResult


class RecognizePeopleSM(ActionSMBase):
    def __init__(self, config_file, timeout=120.0,
                 max_recovery_attempts=1):
        super(RecognizePeopleSM, self).__init__('RecognizePeople', [], max_recovery_attempts)
        self.timeout = timeout

    def init(self):
        rospy.loginfo('[recognize_people] initiialising')
        return FTSMTransitions.INITIALISED

    def running(self):
        rospy.loginfo('[recognize_people] Recognizing people')
        self.result = self.set_result()
        return FTSMTransitions.DONE

    def set_result(self):
        result = RecognizePeopleResult()
        return result

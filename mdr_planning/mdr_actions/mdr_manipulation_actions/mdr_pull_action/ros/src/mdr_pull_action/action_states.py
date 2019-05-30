#!/usr/bin/python
import rospy

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_pull_action.msg import PullGoal, PullResult

class PullSM(ActionSMBase):
    def __init__(self, timeout=120.0, max_recovery_attempts=1):
        super(PullSM, self).__init__('Pull', [], max_recovery_attempts)
        self.timeout = timeout

    def init(self):
        try:
            ## TODO: fill this with any initialisation logic necessary for the action
            pass
        except Exception as exc:
            rospy.logerr('[pull] %s', str(exc))
            return FTSMTransitions.INIT_FAILED
        return FTSMTransitions.INITIALISED

    def running(self):
        ## TODO: fill this method with the execution logic
        return FTSMTransitions.DONE

    def recovering(self):
        ## TODO: if recovery behaviours are appropriate, fill this method with
        ## the recovery logic
        rospy.sleep(5.)
        return FTSMTransitions.DONE_RECOVERING

    def set_result(self, success):
        result = PullResult()
        result.success = success
        return result

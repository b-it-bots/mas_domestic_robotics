#!/usr/bin/python
import rospy

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_push_action.msg import PushGoal, PushResult

class PushSM(ActionSMBase):
    def __init__(self, timeout=120.0, 
                 max_recovery_attempts=1,                
                 gripper_controller_pkg_name='mas_hsr_gripper_controller',
                 move_arm_server='move_arm_server',
                 move_base_server='move_base_server',
                 move_forward_server='move_forward_server',
                 base_elbow_offset=-1.,
                 arm_base_offset=-1.,
                 grasping_dmp='',
                 dmp_tau=1.,
                 number_of_retries=0,
                 ):
        super(PushSM, self).__init__('Push', [], max_recovery_attempts)
        self.timeout = timeout

    def init(self):
        try:
            ## TODO: fill this with any initialisation logic necessary for the action
            pass
        except Exception as exc:
            rospy.logerr('[push] %s', str(exc))
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
        result = PushResult()
        result.success = success
        return result

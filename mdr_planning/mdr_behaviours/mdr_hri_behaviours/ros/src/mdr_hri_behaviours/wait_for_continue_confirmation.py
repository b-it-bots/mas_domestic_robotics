import time

import rospy
from std_msgs.msg import Bool

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class WaitForContinueConfirmation(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'wait_for_continue_confirmation',
                                   save_sm_state=save_sm_state,
                                   outcomes=['continue', 'stop'])
        self.timeout = kwargs.get('timeout', 120.)
        self.proceed = None
        self.proceed_sub = rospy.Subscriber('/continue', Bool, self.proceed_cb)

    def execute(self, userdata):
        self.proceed = None
        rospy.loginfo('Waiting for confirmation to continue...')
        self.say('Waiting for confirmation to continue...')

        start_time = time.time()
        duration = 0.
        while self.proceed is None and duration < self.timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.proceed:
            return 'continue'
        return 'stop'

    def proceed_cb(self, msg):
        if msg.data:
            self.proceed = True
        self.proceed = False

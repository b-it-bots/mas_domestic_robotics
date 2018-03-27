import time
import rospy

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
import diagnostic_msgs.msg as diag_msgs

from mdr_store_groceries.scenario_states.scenario_state_base import ScenarioStateBase

class Place(ScenarioStateBase):
    def __init__(self, **kwargs):
        ScenarioStateBase.__init__(self, 'place',
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)

    def execute(self, userdata):
        success = True
        # TODO: call an action for placing an object
        if success:
            return 'succeeded'

        if self.retry_count == self.number_of_retries:
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'

    def get_dispatch_msg(self):
        pass

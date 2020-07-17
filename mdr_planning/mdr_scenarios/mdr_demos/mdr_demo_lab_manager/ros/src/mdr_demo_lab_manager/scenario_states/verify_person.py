import random

import rospy
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class VerifyPerson(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'verify_person',
                                   save_sm_state=save_sm_state,
                                   outcomes=['no_empty_spot', 'new_person',
                                             'already_logged_person',
                                             'known_person'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'verify_person')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.total_locations = kwargs.get('total_locations', 0)
        self.retry_count = 0
        self.timeout = 120.

    def execute(self, userdata):
        rospy.loginfo('[verify_person] Verifying person')

        occupied_spots = self.kb_interface.get_all_attributes('occupied_location')
        if len(occupied_spots) == self.total_locations:
            return 'no_empty_spot'

        spot = random.randint(1, self.total_locations)
        userdata.destination_locations = ['spot_{0}'.format(spot)]
        return 'new_person'

import random

import rospy
import numpy as np
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from rosplan_knowledge_msgs.msg import DomainFormula
from mas_perception_msgs.msg import Person
from diagnostic_msgs.msg import KeyValue

class VerifyPerson(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'verify_person',
                                   save_sm_state=save_sm_state,
                                   outcomes=['no_empty_spot', 'new_person',
                                             'already_logged_person',
                                             'known_person'],
                                   output_keys=['destination_locations'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'verify_person')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.total_locations = kwargs.get('total_locations', 0)
        self.threshold = kwargs.get('threshold', 0)
        self.retry_count = 0
        self.timeout = 120.
        occupied_locations = self.kb_interface.get_obj_instance('occupied_locations', DomainFormula._type)
        if occupied_locations is None:
            self.kb_interface.insert_obj_instance('occupied_locations', DomainFormula())

    def execute(self, userdata):
        rospy.loginfo('[verify_person] Verifying person')

        # Look for occupied spots
        occupied_locations = self.kb_interface.get_obj_instance('occupied_locations', DomainFormula._type)
        occ_spots = [kv.key for kv in occupied_locations.typed_parameters]
        if len(occupied_locations.typed_parameters) == self.total_locations:
            return 'no_empty_spot'

        # Try to match face to current people in the lab
        known_faces = self.kb_interface.get_obj_of_type(Person._type)
        unknown_face = self.kb_interface.get_obj_instance('person_0', Person._type)

        for known_face in known_faces:
            distance = known_face.face - unknown_face.face
            if np.linalg.norm(distance) < self.threshold: 
                self.say("Welcome back {0}".format(known_face.name))
                return 'known_person'

        # No matching face, treat as new person
        spots = [str(i+1) for i in range(self.total_locations)]
        for occ_spot in occ_spots:
            spots.remove(occ_spot)
        spot = random.choice(spots)
        userdata.destination_locations = ['spot_{0}'.format(spot)]
        occupied_locations.typed_parameters.append(KeyValue(key=str(spot), value='true'))
        self.kb_interface.update_obj_instance('occupied_locations', occupied_locations)
        return 'new_person'

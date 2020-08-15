import random
import numpy as np

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from rosplan_knowledge_msgs.msg import DomainFormula
from mdr_listen_action.msg import ListenAction, ListenGoal
from mas_perception_msgs.msg import Person
from diagnostic_msgs.msg import KeyValue

from mas_execution_manager.scenario_state_base import ScenarioStateBase

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
        self.timeout = kwargs.get('timeout', 10.)
        occupied_locations = self.kb_interface.get_obj_instance('occupied_locations', DomainFormula._type)
        if occupied_locations is None:
            self.kb_interface.insert_obj_instance('occupied_locations', DomainFormula())

        # wait for listen action server
        self.listen_client = actionlib.SimpleActionClient('listen_server', ListenAction)
        listen_wait_result = self.listen_client.wait_for_server(timeout=rospy.Duration(self.timeout))

        if not listen_wait_result:
            raise RuntimeError('Failed to wait for "listen_server" action')

    def execute(self, userdata):
        rospy.loginfo('[verify_person] Verifying person')

        # Look for occupied spots
        occupied_locations = self.kb_interface.get_obj_instance('occupied_locations', DomainFormula._type)
        occ_spots = [kv.key for kv in occupied_locations.typed_parameters]
        if len(occupied_locations.typed_parameters) == self.total_locations:
            return 'no_empty_spot'

        # Try to match face to current people in the lab
        known_people = self.kb_interface.get_obj_of_type(Person._type)
        unknown_person = self.kb_interface.get_obj_instance('person_0', Person._type)

        unknown_person_embedding = np.array(unknown_person.face.views[0].embedding.embedding)
        for known_person in known_people:
            distance = np.linalg.norm(np.array(known_person.face.views[0].embedding.embedding) - \
                                      unknown_person_embedding)

            if np.linalg.norm(distance) < self.threshold:
                self.say("Hello {0}".format(known_person.name))
                self.say("If you would like to free up your spot, please say goodbye.")
                # wait for goodbye
                goal = ListenGoal()

                self.listen_client.send_goal(goal)
                self.listen_client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
                listen_state = self.listen_client.get_state()
                listen_result = self.listen_client.get_result()

                if listen_state == GoalStatus.SUCCEEDED and "bye" in listen_result.message:
                    self.say("Goodbye! {0}. stay safe!".format(known_person.name))
                    spot = known_person

                    for occ_spot in occupied_locations.typed_parameters:
                        if occ_spot.value == known_person.name:
                            occupied_locations.typed_parameters.remove(occ_spot)

                    self.kb_interface.update_obj_instance('occupied_locations', occupied_locations)
                    self.kb_interface.remove_obj_instance(known_person.name, Person._type)

                # otherwise return to monitor door
                return 'known_person'

        # No matching face, treat as new person
        spots = [str(i+1) for i in range(self.total_locations)]
        for occ_spot in occ_spots:
            spots.remove(occ_spot)
        spot = random.choice(spots)
        userdata.destination_locations = ['spot_{0}'.format(spot)]
        # occupied_locations.typed_parameters.append(KeyValue(key=str(spot), value='true'))
        # self.kb_interface.update_obj_instance('occupied_locations', occupied_locations)
        return 'new_person'

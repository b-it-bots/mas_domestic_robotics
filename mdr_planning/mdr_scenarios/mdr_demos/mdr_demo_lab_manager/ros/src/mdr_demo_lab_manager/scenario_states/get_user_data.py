import time
import pandas as pd

import rospy
from diagnostic_msgs.msg import KeyValue
from rosplan_knowledge_msgs.msg import DomainFormula
from mas_perception_msgs.msg import Person
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class GetUserData(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'get_user_data',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'],
                                   input_keys=['destination_locations'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'get_user_data')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        self.timeout = kwargs.get('timeout', 120.)

        self._sheet_id = kwargs.get('sheet_id', '')
        self._worksheet_name = kwargs.get('worksheet_name', 'responses')

        self._num_known_entries = None
        self._loop_rate_s = kwargs.get('loop_rate_s', 2.)

        data = self._load_spreadsheet()
        if data:
            self._num_known_entries = data.shape[0]
            rospy.loginfo("[get_user_data] Initialized the user data query with {0} prior known users".format(self._num_known_entries))
        else:
            rospy.logerr("[get_user_data] Could not initialize user data sheet!")

    def execute(self, userdata):
        if self._num_known_entries is None:
            rospy.logerr("User data was not initialized. Cannot process user input!")
            return "succeeded"

        self.say("Hi, seems like you are visiting the lab for the first time. "
                 "Please enter your details by scanning the QR code displayed on the wall to your right.")

        rospy.loginfo("[get_user_data] waiting for user data")
        start_time = time.time()
        while True and time.time() - start_time < self.timeout:
            data = self._load_spreadsheet()
            num_of_new_entries = data.shape[0] - self._num_known_entries
            if num_of_new_entries > 0:
                self._num_known_entries = data.shape[0]
                new_entry = data[-1, 1:3]
                name, email = new_entry[0], new_entry[1]
                rospy.loginfo("[get_user_data] Found a new entry!\n\tName: {0}\n\tEmail: {1}".format(name, email))

                # we add the new person to permanent storage (one-shot learning of people)
                # and remove it from temporary storage
                person_msg = self.kb_interface.get_obj_instance('person_0', Person._type)
                person_msg.name = name
                self.kb_interface.insert_obj_instance(name, person_msg, permanent_storage=True)
                self.kb_interface.remove_obj_instance('person_0', Person._type)

                # the occupied spots are updated as well
                spot = userdata.destination_locations
                occupied_locations = self.kb_interface.get_obj_instance('occupied_locations',
                                                                        DomainFormula._type)
                occupied_locations.typed_parameters.append(KeyValue(key=str(spot), value=name))
                self.kb_interface.update_obj_instance('occupied_locations', occupied_locations)

                return "succeeded"
            else:
                time.sleep(self._loop_rate_s)
        rospy.logerr("[get_user_data] Query timed out!")

        if self.retry_count == self.number_of_retries:
            self.say("Sorry, I could not get your name. I shall proceed without it.")
            self.retry_count = 0
            return "failed_after_retrying"

        self.say("Could you please enter your data? I still have not received it.")
        self.retry_count += 1
        return "failed"

    def _load_spreadsheet(self):
        URL = 'https://docs.google.com/spreadsheets/d/{0}/gviz/tq?tqx=out:csv&sheet={1}'.format(self._sheet_id,
                                                                                                self._worksheet_name)
        try:
            data = pd.read_csv(URL, sep=",").to_numpy()
            return data
        except Exception as exc:
            rospy.logerr(str(exc))
            return None

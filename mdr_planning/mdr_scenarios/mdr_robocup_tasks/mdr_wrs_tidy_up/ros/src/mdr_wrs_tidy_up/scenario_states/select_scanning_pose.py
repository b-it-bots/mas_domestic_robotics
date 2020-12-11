import random
from mas_execution_manager.scenario_state_base import ScenarioStateBase

from mdr_wrs_tidy_up.utils import reconfigure_object_detection_params

def get_dict_keys_with_false_values(d):
    keys_with_false_values = []
    for k, v in d.items():
        if not v:
            keys_with_false_values.append(k)
    return keys_with_false_values

class SelectScanningPose(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'select_scanning_pose',
                                   save_sm_state=save_sm_state,
                                   input_keys=['floor_objects_cleared',
                                               'table_objects_cleared'],
                                   output_keys=['destination_locations',
                                                'object_location'],
                                   outcomes=['floor_not_cleared',
                                             'table_not_cleared',
                                             'tidying_done'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'select_scanning_pose')

    def execute(self, userdata):
        dirty_locations = []
        if False in userdata.floor_objects_cleared.values():
            dirty_locations = get_dict_keys_with_false_values(userdata.floor_objects_cleared)
            userdata.object_location = 'floor'
            userdata.destination_locations = [random.choice(dirty_locations)]
            reconfigure_object_detection_params(0.02, 0.2)
            return 'floor_not_cleared'
        elif False in userdata.table_objects_cleared.values():
            dirty_locations = get_dict_keys_with_false_values(userdata.table_objects_cleared)
            userdata.object_location = 'table'
            userdata.destination_locations = [random.choice(dirty_locations)]
            reconfigure_object_detection_params(0.4, 0.6)
            return 'table_not_cleared'
        else:
            return 'tidying_done'

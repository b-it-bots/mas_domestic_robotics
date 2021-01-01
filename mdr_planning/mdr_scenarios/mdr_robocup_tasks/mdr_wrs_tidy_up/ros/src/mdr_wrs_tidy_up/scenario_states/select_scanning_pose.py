import random

import rospy
from mas_execution_manager.scenario_state_base import ScenarioStateBase

from mdr_wrs_tidy_up.utils import update_object_detection_params

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
                                               'table_objects_cleared',
                                               'operation_start_time'],
                                   output_keys=['destination_locations',
                                                'object_location'],
                                   outcomes=['floor_not_cleared',
                                             'table_not_cleared',
                                             'tidying_done'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'select_scanning_pose')
        self.tidying_timeout_s = kwargs.get('tidying_timeout_s', 900.)
        self.avg_obj_cleanup_duration_s = kwargs.get('avg_obj_cleanup_duration_s', 90.)

    def execute(self, userdata):
        current_time = rospy.Time.now().to_sec()
        operation_duration = current_time - userdata.operation_start_time
        if (operation_duration + self.avg_obj_cleanup_duration_s) > self.tidying_timeout_s:
            rospy.loginfo('[%s] Interrupting tidying up in order not to exceed timeout of %f s',
                          self.state_name, self.tidying_timeout_s)
            return 'tidying_done'

        rospy.loginfo('[%s] Will carry on tidying up; used %f seconds so far',
                      self.state_name, operation_duration)
        dirty_locations = []
        if False in userdata.floor_objects_cleared.values():
            dirty_locations = get_dict_keys_with_false_values(userdata.floor_objects_cleared)
            userdata.object_location = 'floor'
            userdata.destination_locations = [random.choice(dirty_locations)]
            update_object_detection_params("floor")
            return 'floor_not_cleared'
        elif False in userdata.table_objects_cleared.values():
            dirty_locations = get_dict_keys_with_false_values(userdata.table_objects_cleared)
            userdata.object_location = 'table'
            destination_location = random.choice(dirty_locations)
            userdata.destination_locations = [destination_location]
            update_object_detection_params(destination_location)
            return 'table_not_cleared'
        else:
            return 'tidying_done'

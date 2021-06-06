import rospy
from mas_execution_manager.scenario_state_base import ScenarioStateBase

from mdr_wrs_tidy_up.utils import update_object_detection_params


class ConfigureObjectDetection(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'configure_object_detection',
                                   save_sm_state=save_sm_state,
                                   input_keys=[],
                                   output_keys=['destination_locations',
                                                'object_location'],
                                   outcomes=['succeeded'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'configure_object_detection')

    def execute(self, userdata):
        destination_location = 'shelf'
        ## Expected by FIND_OBJECTS state:
        userdata.destination_locations = [destination_location]
        ## Required to configure cloud detector 
        ## Failing to do this leads to last_cloud_object_detection_time never being set and a runtime error.
        update_object_detection_params(destination_location)
        return 'succeeded'

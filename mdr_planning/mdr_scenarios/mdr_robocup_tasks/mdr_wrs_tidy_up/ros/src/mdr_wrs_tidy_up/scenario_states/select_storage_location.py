import random
import rospy
from mas_tools.ros_utils import get_package_path
from mas_tools.file_utils import load_yaml_file
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class SelectStorageLocation(ScenarioStateBase):
    storage_locations_map = None

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'select_storage_location',
                                   save_sm_state=save_sm_state,
                                   input_keys=['grasped_object'],
                                   output_keys=['destination_locations',
                                                'storage_location'],
                                   outcomes=['succeeded'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'select_storage_location')
        self.storage_locations_file = kwargs.get('storage_locations_file', 'storage_location.yaml')

        self.available_trays = kwargs.get('available_trays', ['tray_1', 'tray_2'])
        self.tray_locations = kwargs.get('tray_locations', {'tray_1': 'long_table_a_trays',
                                                            'tray_2': 'long_table_a_trays'})
        self.available_bins = kwargs.get('available_bins', ['bin_a', 'bin_b'])
        self.bin_locations = kwargs.get('bin_locations', {'bin_a': 'bin_a',
                                                          'bin_b': 'bin_b'})
        self.default_storage_location = kwargs.get('default_storage_location', 'bin')

        storage_locations_path = get_package_path('mdr_wrs_tidy_up', 'config', self.storage_locations_file)
        if not self.storage_locations_file:
            raise ValueError('[{0}] Argument "storage_locations_file" cannot be empty'.format(self.state_name))
        self.storage_locations_map = load_yaml_file(storage_locations_path)

    def execute(self, userdata):
        obj_category = userdata.grasped_object.category
        storage_location = self.default_storage_location
        if obj_category and obj_category in self.storage_locations_map:
            storage_location = self.storage_locations_map[obj_category]

        if 'table' in storage_location:
            storage_location = random.choice(self.available_trays)
            userdata.storage_location = storage_location
            userdata.destination_locations = [self.tray_locations[storage_location]]
        else:
            storage_location = random.choice(self.available_bins)
            userdata.storage_location = storage_location
            userdata.destination_locations = [self.bin_locations[storage_location]]
            rospy.logwarn('STORAGE LOCATION %s', storage_location)
            rospy.logwarn([self.bin_locations[storage_location]])
        return 'succeeded'

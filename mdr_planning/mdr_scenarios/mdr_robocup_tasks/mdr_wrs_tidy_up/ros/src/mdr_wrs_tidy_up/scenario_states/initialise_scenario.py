from mas_execution_manager.scenario_state_base import ScenarioStateBase

class InitialiseScenario(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'initialise_scenario',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded'],
                                   output_keys=['floor_objects_cleared',
                                                'table_objects_cleared',
                                                'object_location'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'initialise_scenario')
        self.floor_objects_cleared = kwargs.get('floor_objects_cleared', None)
        self.table_objects_cleared = kwargs.get('table_objects_cleared', None)
        self.object_location = kwargs.get('object_location', 'floor')

    def execute(self, userdata):
        userdata.floor_objects_cleared = self.floor_objects_cleared
        userdata.table_objects_cleared = self.table_objects_cleared
        userdata.object_location = self.object_location
        return 'succeeded'

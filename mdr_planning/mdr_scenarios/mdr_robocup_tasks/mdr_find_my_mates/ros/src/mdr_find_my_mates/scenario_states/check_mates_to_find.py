from mas_execution_manager.scenario_state_base import ScenarioStateBase

class CheckMatesToFind(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'check_mates_to_find',
                                   save_sm_state=save_sm_state,
                                   outcomes=['mates_left', 'no_mates_left'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'check_mates_to_find')

    def execute(self, userdata):
        unknown_instances = self.kb_interface.get_all_attributes('unknown')
        if unknown_instances:
            return 'mates_left'
        return 'no_mates_left'

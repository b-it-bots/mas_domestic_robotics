from mas_execution_manager.scenario_state_base import ScenarioStateBase

class CheckEmptySurface(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'check_empty_surface',
                                   save_sm_state=save_sm_state,
                                   outcomes=['empty', 'not_empty'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'check_empty_surface')
        self.surface_prefix = kwargs.get('surface_prefix', '')

    def execute(self, userdata):
        if self.kb_interface.is_surface_empty(self.surface_prefix):
            self.say('{0} is empty'.format(self.surface_prefix))
            return 'empty'
        return 'not_empty'

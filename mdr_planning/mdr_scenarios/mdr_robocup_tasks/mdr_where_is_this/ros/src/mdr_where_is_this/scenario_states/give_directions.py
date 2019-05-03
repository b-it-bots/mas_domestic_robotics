from mas_execution_manager.scenario_state_base import ScenarioStateBase

class GiveDirections(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'give_directions',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'give_directions')
        self.number_of_retries = kwargs.get('number_of_retries', 0)

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        self.say("Give Direction state.")

        return 'succeeded'

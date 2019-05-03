from mas_execution_manager.scenario_state_base import ScenarioStateBase

class ReceiveInformation(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receive_information',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'receive_information')
        self.number_of_retries = kwargs.get('number_of_retries', 0)

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        self.say("Receive information state.")

        return 'succeeded'

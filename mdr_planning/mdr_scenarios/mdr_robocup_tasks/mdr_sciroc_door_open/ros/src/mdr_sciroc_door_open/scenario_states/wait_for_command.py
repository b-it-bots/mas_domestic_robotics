from mas_execution_manager.scenario_state_base import ScenarioStateBase


class WaitForCommand(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'wait_for_command',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'wait_for_command')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)

    def execute(self, userdata):
        self.say('waiting for command from the data hub')
        return 'succeeded'


class SimulatedGoTo(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'simulated_go_to',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])

        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'simulated_go_to')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)

        self.destination_locations = list(kwargs.get('destination_locations', list()))

    def execute(self, userdata):
        for dest in self.destination_locations:
            self.say('going to ' + dest)
        return 'succeeded'

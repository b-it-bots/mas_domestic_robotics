from mas_execution_manager.scenario_state_base import ScenarioStateBase

class DescribeLocation(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'describe_location',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'describe_location')
        self.number_of_retries = kwargs.get('number_of_retries', 0)

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        self.say("Describe location state.")

        return 'succeeded'

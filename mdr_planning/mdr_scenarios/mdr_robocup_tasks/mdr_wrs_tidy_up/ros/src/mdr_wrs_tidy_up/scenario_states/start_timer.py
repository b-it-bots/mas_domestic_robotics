from mas_execution_manager.scenario_state_base import ScenarioStateBase

class StartTimer(ScenarioStateBase):

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'start_timer',
                                   save_sm_state=save_sm_state,
                                   output_keys=['operation_start_time'],
                                   outcomes=['succeeded'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'start_timer')

    def execute(self, userdata):
        userdata.operation_start_time = rospy.Time.now().to_sec()
        return 'succeeded'

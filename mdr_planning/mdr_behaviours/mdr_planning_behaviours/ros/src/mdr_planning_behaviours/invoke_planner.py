from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_rosplan_interface.planner_interface import PlannerInterface

class InvokePlanner(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'invoke_planner',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'invoke_planner')
        self.planner_interface = PlannerInterface()

    def execute(self, userdata):
        planner_called = self.planner_interface.plan()
        if planner_called:
            return 'succeeded'
        return 'failed'

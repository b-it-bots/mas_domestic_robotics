from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_rosplan_interface.planner_interface import PlannerInterface

class DispatchPlan(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'dispatch_plan',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'dispatch_plan')
        self.planner_interface = PlannerInterface()

    def execute(self, userdata):
        dispatching = self.planner_interface.start_plan_dispatch()
        if not dispatching:
            return 'failed'

        # TODO: monitor the plan
        return 'succeeded'

import rospy

from mdr_move_base_action.msg import ListenAction, ListenGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class Listen(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'listen',
                                   save_sm_state=save_sm_state,
                                   outcomes=['received_command', 'failed'])



        self.listen_client = actionlib.SimpleActionClient('listen_server', ListenAction)
        self.listen_client.wait_for_server()


    def execute(self, userdata):
        listen_goal = ListenGoal()
        self.listen_client.send_goal(listen_goal)

        timeout = 15.0
        self.listen_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        result = self.listen_client.get_result()
        if result.success:
            return 'received_command'
        else:
            return 'failed'

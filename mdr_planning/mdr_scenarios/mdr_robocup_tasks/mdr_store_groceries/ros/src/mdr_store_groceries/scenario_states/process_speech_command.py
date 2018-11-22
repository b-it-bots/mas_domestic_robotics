import rospy
import actionlib

from mdr_process_speech_command_action.msg import ProcessSpeechAction, ProcessSpeechGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class ProcessCommand(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'listen',
                                   save_sm_state=save_sm_state,
                                   input_keys=['listen_result'],
                                   outcomes=['start_store_groceries', 'continue_waiting'])

        self.start_command = kwargs.get('start_command', 'store groceries')
        self.process_command_server = kwargs.get('process_command_server',
                                                 '/mdr_actions/process_speech_command_server')
        self.process_command_client = actionlib.SimpleActionClient(self.process_command_server,
                                                                   ProcessSpeechAction)
        self.process_command_client.wait_for_server()

    def execute(self, userdata):
        process_command_goal = ProcessSpeechGoal()
        process_command_goal.command = userdata.listen_result.message
        process_command_goal.start_command = self.start_command
        self.process_command_client.send_goal(process_command_goal)

        timeout = 15.0
        self.process_command_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        result = self.process_command_client.get_result()
        if result and result.success:
            return 'start_store_groceries'
        else:
            return 'continue_waiting'

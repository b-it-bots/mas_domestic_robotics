import rospy
import actionlib

from mdr_process_speech_command_action.msg import ProcessSpeechAction, ProcessSpeechGoal
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class ProcessCommand(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'listen',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])



        self.process_speech_command_client = actionlib.SimpleActionClient('process_speech_command_server', ProcessSpeechAction)
        self.process_speech_command_client.wait_for_server()

    def execute(self, userdata):
        process_speech_command_goal = ExecutionGoal()
        process_speech_command_goal.command = userdata.listen_result.message
        self.process_speech_command_client.send_goal(process_speech_command_goal)

        timeout = 15.0
        self.process_speech_command_client.wait_for_result(rospy.Duration.from_sec(int(timeout)))
        result = self.process_speech_command_client.get_result()
        if result.success:
            return 'start_store_groceries'
        else:
            return 'continue_waiting'

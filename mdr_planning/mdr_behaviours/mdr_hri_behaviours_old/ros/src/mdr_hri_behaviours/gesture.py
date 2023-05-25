import rospy
import actionlib

from mas_execution_manager.scenario_state_base import ScenarioStateBase

from mdr_listen_action.msg import ListenAction, ListenGoal
from mas_tools.ros_utils import get_package_path

class Gesture(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'gesture',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['command'])
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 120.)

        #self.threshold = kwargs.get('threshold', 0.68)


        # Load rasa nlu model
        #model_directory = get_package_path('rasa_nlu_models','common', 'mdr_hri_behaviours_models','nlu')
        #self.interpreter = Interpreter.load(model_directory)

    def execute(self, userdata):
        rospy.loginfo('Retrieving information from operator ...')

        self.say('Please wait, I am waiting for listen action server')

        client = actionlib.SimpleActionClient("listen_server", ListenAction)
        client.wait_for_server()
        goal = ListenGoal()

        self.say('What should I look for?')
        
        ongoing_conversation = True
        while ongoing_conversation :
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
            recognized_speech = client.get_result()

            print(recognized_speech.message)

            txt= str(recognized_speech.message)

            self.say("Did you say this")

            self.say(txt)

            ongoing_conversation = False

        if not ongoing_conversation:
            return 'succeeded'
        return 'failed'

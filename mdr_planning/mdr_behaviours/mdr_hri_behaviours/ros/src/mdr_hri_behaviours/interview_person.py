import rospy
import actionlib
import rospkg

from mas_execution_manager.scenario_state_base import ScenarioStateBase

from rasa_nlu.training_data import load_data
from rasa_nlu import config
from rasa_nlu.components import ComponentBuilder
from rasa_nlu.model import Interpreter, Metadata, Trainer

from mdr_listen_action.msg import ListenAction, ListenGoal

class Interview(ScenarioStateBase):

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'interview',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'interview')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 120.)

        self.threshold = kwargs.get('threshold', 0.68)

        rospack = rospkg.RosPack()
        package_directory = rospack.get_path("mdr_hri_behaviours")
        model_directory = (package_directory + '/common/models/')               # MODEL DIRECTORY
        self.interpreter = Interpreter.load(model_directory)

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        client = actionlib.SimpleActionClient("listen_server", ListenAction)
        client.wait_for_server()
        goal = ListenGoal()

        self.succeeded = False
        repeated_questions = 0
        self.say('Hi. What is your name?')
        ongoing_conversation = True
        while ongoing_conversation or repeated_questions < 3:
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
            understood_voice = client.get_result()
            result = self.interpreter.parse(understood_voice.message)
            intent_of_result = result["intent"]
            if intent_of_result["name"] == "name" and intent_of_result["confidence"] >= self.threshold:
                # UPDATE DATABASE
                ongoing_conversation = False
                self.succeeded = True
            elif intent_of_result["name"] == "name" and intent_of_result["confidence"] < self.threshold:
                repeated_questions += 1
                self.say("I did not understand you. Could you repeat please?")

        if self.succeeded:
            rospy.loginfo('Person interviewed successfully')
            self.say('Thank you! Have a nice day!')
            return 'succeeded'

        rospy.loginfo('Could not retrieve the right information.')
        self.say('Sorry. I could not understand you!')
        rospy.loginfo('Failed to interview person')
        self.say('Aborting operation')
        return 'failed_after_retrying'

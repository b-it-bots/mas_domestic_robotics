import rospy
from std_msgs.msg import String
from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_perception_msgs.msg import PersonInfo
from rasa_nlu.training_data import load_data
from rasa_nlu import config
from rasa_nlu.components import ComponentBuilder
from rasa_nlu.model import Interpreter, Metadata, Trainer
from mdr_listen_action.msg import ListenAction, ListenGoal

class RetrieveInformation(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'retrieve_information',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'],
                                   input_keys=['command'])
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 120.)

        self.threshold = kwargs.get('threshold', 0.68)


        rospack = rospkg.RosPack()
        package_directory = rospack.get_path("mdr_hri_behaviours")
        model_directory = (package_directory + '/common/models/')               # MODEL DIRECTORY
        self.interpreter = Interpreter.load(model_directory)

    def execute(self, userdata):
        rospy.loginfo('Retrieving information from operator ...')

        client = actionlib.SimpleActionClient("listen_server", ListenAction)
        client.wait_for_server()
        goal = ListenGoal()

        self.say('Who should I look for?')
        try:
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
            recognized_speech = client.get_result()

            result = self.interpreter.parse(understood_voice.message)
            intent_of_result = result["intent"]
            if intent_of_result["name"] == "name" and intent_of_result["confidence"] >= self.threshold:
                # UPDATE DATABASE
                ongoing_conversation = False
                self.succeeded = True
            elif intent_of_result["name"] == "name" and intent_of_result["confidence"] < self.threshold:
                repeated_questions += 1
                self.say("I did not understand you. Could you repeat please?")

        except Exception as ex:
            rospy.logerr(type(ex).__name__ + ": You have failed!")
            return 'failed'

        return 'succeeded'

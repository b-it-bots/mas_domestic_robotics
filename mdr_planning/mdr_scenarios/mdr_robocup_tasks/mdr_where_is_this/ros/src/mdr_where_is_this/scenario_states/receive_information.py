import os
import rospy
import actionlib
import rospkg

from mas_execution_manager.scenario_state_base import ScenarioStateBase

from rasa_nlu.training_data import load_data
from rasa_nlu import config
from rasa_nlu.components import ComponentBuilder
from rasa_nlu.model import Interpreter, Metadata, Trainer

from mdr_listen_action.msg import ListenAction, ListenGoal

class ReceiveInformation(ScenarioStateBase):

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'receive_information',
                                   save_sm_state=save_sm_state,
                                   input_keys=[],
                                   output_keys=['target_entities'],
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'])

        # Get parameters
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'receive_information')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.timeout = kwargs.get('timeout', 25)
        self.threshold = kwargs.get('threshold', 0.68)

        # Load the rasa model
        rospack = rospkg.RosPack()
        package_directory = rospack.get_path("mdr_where_is_this")
        model_directory = os.path.join(package_directory, 'common', 'model', 'current', 'nlu')
        self.interpreter = Interpreter.load(model_directory)

        # wait for listen action server
        self.client = actionlib.SimpleActionClient("listen_server", ListenAction)
        wait_result = self.client.wait_for_server(timeout=rospy.Duration(self.timeout))
        if not wait_result:
            raise RuntimeError('failed to wait for "listen_server" action')

    def execute(self, userdata):
        # Setup the listen goal
        goal = ListenGoal()

        self.say('Hi. What are you looking for?')

        for times_repeated in range(self.number_of_retries):
            # Ask again if not understood the first time
            if times_repeated > 0:
                self.say("Sorry, I didn't understand you. Could you repeat that please?")

            # Listen for speech
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
            listen_result = self.client.get_result()

            # Try to get the intent
            rasa_result = self.interpreter.parse(listen_result.message)
            speech_intent = rasa_result["intent"]

            # Put relevant intents into a list
            target_entities = []
            for entity in rasa_result["entities"]:
                entity_type = entity['entity']
                entity_value = entity['value']
                if entity['entity'] == 'location' or entity['entity'] == 'person':
                    target_entities.append({'type': entity_type, 'value': entity_value})

            # If we are confident the operator asked for an object location
            if speech_intent["name"] == "question" and speech_intent["confidence"] >= self.threshold and len(target_entities) >= 1:
                userdata['target_entities'] = target_entities
                rospy.loginfo('Successfully got wanted object/location: {}'.format(target_entities))
                self.say('Thank you! I will describe you now how to get there.')
                return 'succeeded'

            # If we are unsure about what we understood
            else:
                rospy.logerr('Could not understand human:')

                if speech_intent["name"] != "question":
                    rospy.logerr(' -> Intent is not question')

                if speech_intent["confidence"] < self.threshold:
                    rospy.logerr(' -> Confidence for the intent is too low: {}'.format(speech_intent["confidence"]))

                if len(rasa_result["entities"]) < 1:
                    rospy.logerr(' -> Could not extract object entity')


        # Conversation has exceeded the maximum number of retries at this point
        rospy.loginfo('Could not retrieve the wanted object/location!')
        self.say('Sorry, I definitively cannot understand you! Aborting operation!')

        return 'failed_after_retrying'

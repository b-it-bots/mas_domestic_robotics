import rospy
import actionlib
import rospkg

from mas_perception_msgs.msg import Person
from mdr_perception_msgs.msg import PersonInfo
from mas_execution_manager.scenario_state_base import ScenarioStateBase


from mdr_listen_action.msg import ListenAction, ListenGoal

class InterviewPerson(ScenarioStateBase):

    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'interview',
                                   save_sm_state=save_sm_state,
                                   input_keys=['person_interviewing'],
                                   output_keys=['person_name'],
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

        person_identifier = userdata.person_interviewing
        known_male_names = [x.lower() for x in self.ontology_interface.get_instances_of('Male')]
        known_female_names = [x.lower() for x in self.ontology_interface.get_instances_of('Female')]

        client = actionlib.SimpleActionClient("listen_server", ListenAction)
        client.wait_for_server()
        goal = ListenGoal()

        self.succeeded = False
        repeated_questions = 0
        self.say('Hi. What is your name?')
        ongoing_conversation = True
        name = None
        while ongoing_conversation or repeated_questions < 3:
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration.from_sec(int(self.timeout)))
            understood_voice = client.get_result()
            result = self.interpreter.parse(understood_voice.message)
            intent_of_result = result["intent"]
            if intent_of_result["name"] == "greeting" and intent_of_result["confidence"] >= self.threshold:
                name = result["entities"][0]["entity"].lower()

                facts_to_remove = [('unknown', [('person', person_identifier)])]
                facts_to_add = [('known', [('person', name)])]
                self.kb_interface.update_kb(facts_to_add, facts_to_remove)

                person_data = self.kb_interface.get_obj_instance(person_identifier, Person)
                self.kb_interface.insert_obj_instance(name, person_data)
                self.kb_interface.remove_obj_instance(person_identifier, Person)

                person_info = PersonInfo()
                person_info.name = name

                if name in known_male_names:
                    person_info.gender = 'male'
                elif name in known_female_names:
                    person_info.gender = 'female'
                else:
                    person_info.gender = 'unknown'

                self.kb_interface.insert_obj_instance(name, PersonInfo)

                ongoing_conversation = False
                self.succeeded = True
            elif intent_of_result["name"] != "greeting" or intent_of_result["confidence"] < self.threshold:
                repeated_questions += 1
                self.say("I did not understand you. Could you repeat please?")

        if self.succeeded:
            userdata.person_name = name
            rospy.loginfo('Person interviewed successfully')
            self.say('Thank you! Have a nice day!')
            return 'succeeded'

        rospy.loginfo('Could not retrieve the right information.')
        self.say('Sorry. I could not understand you!')
        rospy.loginfo('Failed to interview person')
        self.say('Aborting operation')
        return 'failed_after_retrying'

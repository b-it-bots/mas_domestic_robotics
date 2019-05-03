from mas_execution_manager.scenario_state_base import ScenarioStateBase

class ProvideLocationInformation(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'provide_location_information',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'provide_location_information')
        self.number_of_retries = kwargs.get('number_of_retries', 0)

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        # Check object in the knowledge base.
        # Query location information from ontology.
        furniture = "shelf"
        room = "living room"
        if furniture != None:
            self.say("The object you are looking for is on " + furniture + " in room " + room)
        else:
            self.say("The object you are looking for is in room " + room)

        rospy.sleep(2)
        self.say("I will provide further description there. Please, let us meet in " + room)

        return 'succeeded'

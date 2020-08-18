from mas_perception_msgs.msg import Person

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class StorePerson(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'store_person',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'retake_picture', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'store_person')
        self.person_msg_id = kwargs.get('person_msg_id', 'person_0')

    def execute(self, userdata):
        person = self.kb_interface.get_obj_instance(self.person_msg_id,
                                                    Person._type)
        if person is None:
            self.say('Sorry, I got confused for some reason, we will have to stop the procedure.')
            self.kb_interface.remove_obj_instance(self.person_msg_id, Person._type)
            return 'failed'

        if not person.face.views[0].embedding.embedding:
            self.say('I unfortunately could not see your face.')
            self.say('Let us try taking the picture again.')
            self.kb_interface.remove_obj_instance(self.person_msg_id, Person._type)
            return 'retake_picture'

        person_permanent_storage = self.kb_interface.get_obj_instance(self.person_msg_id,
                                                                      Person._type,
                                                                      permanent_storage=True)

        # if we already have the person in permanent storage, we add
        # the most recent view of the face to the list of face views;
        # if not, we just insert the complete person message
        # to permanent storage so that we can update it later
        if person_permanent_storage is not None:
            person_permanent_storage.face.views.append(person.face.views[0])
            self.kb_interface.update_obj_instance(self.person_msg_id,
                                                  person_permanent_storage,
                                                  permanent_storage=True)
        else:
            self.kb_interface.insert_obj_instance(self.person_msg_id,
                                                  person,
                                                  permanent_storage=True)

        # we remove the person message from temporary storage
        self.kb_interface.remove_obj_instance(self.person_msg_id, Person._type)
        return 'succeeded'

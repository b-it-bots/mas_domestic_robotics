#!/usr/bin/env python
import rospy
import smach
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface
from mdr_find_object_action.msg import FindObjectGoal, FindObjectResult, FindObjectFeedback

class FindObject(smach.State):
    def __init__(self, ontology_url, ontology_class_prefix,
                 number_of_retries=0, timeout=120.):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'timeout'],
                             input_keys=['find_object_goal'],
                             output_keys=['find_object_feedback',
                                          'obj_location', 'relation'])
        self.ontology_interface = DomesticOntologyInterface(ontology_url,
                                                            ontology_class_prefix)
        self.kb_interface = DomesticKBInterface()

        self.number_of_retries = number_of_retries
        self.timeout = timeout

    def execute(self, userdata):
        if userdata.find_object_goal.goal_type == FindObjectGoal.NAMED_OBJECT:
            obj_name = userdata.find_object_goal.object_name
            location, predicate = self.kb_interface.get_object_location(obj_name)
            if location:
                rospy.loginfo('[FIND_OBJECT] Found %s %s %s', obj_name, predicate, location)
                # TODO: verify that the object is still at that location
                userdata.obj_location = location
                userdata.relation = predicate
                return 'succeeded'

            rospy.loginfo('[FIND_OBJECT] %s not found in the knowledge base; querying the ontology', obj_name)
            location = self.ontology_interface.get_default_storing_location(obj_name=obj_name)
            if location:
                predicate = 'in'
                rospy.loginfo('[FIND_OBJECT] %s is usually %s %s', obj_name, predicate, location)
                # TODO: check if the object is at the default location
                userdata.obj_location = location
                userdata.relation = predicate
                return 'succeeded'

            rospy.loginfo('[FIND_OBJECT] %s not found', obj_name)
            userdata.obj_location = None
            userdata.relation = None
            return 'failed'
        elif userdata.find_object_goal.goal_type == FindObjectGoal.OBJECT_CATEGORY:
            obj_category = userdata.find_object_goal.object_name
            category_objects = self.kb_interface.get_category_objects(obj_category)
            if category_objects:
                for obj_name in category_objects:
                    location, predicate = self.kb_interface.get_object_location(obj_name)
                    if location:
                        rospy.loginfo('[FIND_OBJECT] Found %s %s %s', obj_name, predicate, location)
                        # TODO: verify that the object is still at that location
                        userdata.obj_location = location
                        userdata.relation = predicate
                        return 'succeeded'
                    else:
                        rospy.loginfo('[FIND_OBJECT] The location of %s is unknown', obj_name)
            else:
                rospy.loginfo('[FIND_OBJECT] No object of category %s was found in the knowledge base; querying the ontology', obj_category)

            location = self.ontology_interface.get_default_storing_location(obj_category=obj_category)
            if location:
                predicate = 'in'
                rospy.loginfo('[FIND_OBJECT] Objects of %s are usually %s %s', obj_category, predicate, location)
                # TODO: check if an object of the desired category is at the default location
                userdata.obj_location = location
                userdata.relation = predicate
                return 'succeeded'

            return 'failed'

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['find_object_goal', 'find_object_feedback',
                                         'obj_location', 'relation'],
                             output_keys=['find_object_feedback', 'find_object_result'])
        self.result = result

    def execute(self, userdata):
        if 'find_object_feedback' not in userdata:
            userdata.find_object_feedback = FindObjectFeedback()
        userdata.find_object_feedback.current_state = 'set_action_lib_result'

        result = FindObjectResult()
        result.success = self.result
        result.object_location = userdata.obj_location
        result.relation = userdata.relation
        userdata.find_object_result = result
        return 'succeeded'

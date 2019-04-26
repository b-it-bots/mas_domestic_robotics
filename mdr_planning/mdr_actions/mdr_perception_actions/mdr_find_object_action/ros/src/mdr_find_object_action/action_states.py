#!/usr/bin/env python
import rospy
from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mas_knowledge_utils.domestic_ontology_interface import DomesticOntologyInterface
from mas_knowledge_base.domestic_kb_interface import DomesticKBInterface
from mdr_find_object_action.msg import FindObjectGoal, FindObjectResult

class FindObjectSM(ActionSMBase):
    def __init__(self, ontology_url,
                 ontology_class_prefix,
                 number_of_retries=0,
                 timeout=120.,
                 max_recovery_attempts=1):
        super(FindObjectSM, self).__init__('FindObject', [], max_recovery_attempts)
        self.ontology_url = ontology_url
        self.ontology_class_prefix = ontology_class_prefix
        self.number_of_retries = number_of_retries
        self.timeout = timeout
        self.ontology_interface = None
        self.kb_interface = None

    def init(self):
        try:
            rospy.loginfo('[find_object] Creating an interface client for ontology %s', self.ontology_url)
            self.ontology_interface = DomesticOntologyInterface(self.ontology_url,
                                                                self.ontology_class_prefix)
        except Exception as exc:
            rospy.logerr('[find_object] Could not create an ontology interface client: %s', exc)

        try:
            rospy.loginfo('[find_object] Creating a knowledge base interface client')
            self.kb_interface = DomesticKBInterface()
        except Exception as exc:
            rospy.logerr('[find_object] Could not create a knowledge base interface client: %s', exc)
        return FTSMTransitions.INITIALISED

    def running(self):
        obj_name = None
        obj_location = None
        relation = None
        if self.goal.goal_type == FindObjectGoal.NAMED_OBJECT:
            obj_name = self.goal.object_name
            location, predicate = self.kb_interface.get_object_location(obj_name)
            if location:
                rospy.loginfo('[find_object] Found %s %s %s', obj_name, predicate, location)
                # TODO: verify that the object is still at that location
                obj_location = location
                relation = predicate
                self.result = self.set_result(True, obj_location, relation)
                return FTSMTransitions.DONE

            rospy.loginfo('[find_object] %s not found in the knowledge base; querying the ontology', obj_name)
            location = self.ontology_interface.get_default_storing_location(obj_name=obj_name)
            if location:
                predicate = 'in'
                rospy.loginfo('[find_object] %s is usually %s %s', obj_name, predicate, location)
                # TODO: check if the object is at the default location
                obj_location = location
                relation = predicate
                self.result = self.set_result(True, obj_location, relation)
                return FTSMTransitions.DONE

            rospy.loginfo('[find_object] %s not found', obj_name)
            obj_location = None
            relation = None
            self.result = self.set_result(True, obj_location, relation)
            return FTSMTransitions.DONE
        elif self.goal.goal_type == FindObjectGoal.OBJECT_CATEGORY:
            obj_category = self.goal.object_name
            category_objects = self.kb_interface.get_category_objects(obj_category)
            if category_objects:
                for obj_name in category_objects:
                    location, predicate = self.kb_interface.get_object_location(obj_name)
                    if location:
                        rospy.loginfo('[find_object] Found %s %s %s', obj_name, predicate, location)
                        # TODO: verify that the object is still at that location
                        obj_location = location
                        relation = predicate
                        self.result = self.set_result(True, obj_location, relation)
                        return FTSMTransitions.DONE
                    else:
                        rospy.loginfo('[find_object] The location of %s is unknown', obj_name)
            else:
                rospy.loginfo('[find_object] No object of category %s was found in the knowledge base; querying the ontology', obj_category)

            location = self.ontology_interface.get_default_storing_location(obj_category=obj_category)
            if location:
                predicate = 'in'
                rospy.loginfo('[find_object] Objects of %s are usually %s %s', obj_category, predicate, location)
                # TODO: check if an object of the desired category is at the default location
                obj_location = location
                relation = predicate
                self.result = self.set_result(True, obj_location, relation)
                return FTSMTransitions.DONE

            rospy.logerr('[find_object] Object of category %s could not be found', obj_category)
            self.result = self.set_result(False, obj_location, relation)
            return FTSMTransitions.DONE

    def set_result(self, success, obj_location, relation):
        result = FindObjectResult()
        result.success = success
        result.object_location = obj_location
        result.relation = relation
        return result

'''Module defining interfaces for interacting with a knowledge base.
'''

import rospy
import diagnostic_msgs.msg as diag_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
from mongodb_store.message_store import MessageStoreProxy

from mdr_monitoring_msgs.msg import ExecutionState

class KnowledgeUpdateTypes(object):
    # the values of these constants are set to correspond
    # with the values used by the ROSPlan knowledge update service
    INSERT = 0
    REMOVE = 2

class KnowledgeBaseInterface(object):
    '''Defines an interface for interacting with a knowledge base.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self):
        # a client for the /rosplan_knowledge_base/update service
        self.knowledge_update_client = None

        # a client for the /rosplan_knowledge_base/state/propositions service
        self.attribute_retrieval_client = None

        # a mongodb_store.message_store.MessageStoreProxy instance
        self.msg_store_client = None

        # name of the host robot; value read from the knowledge base
        self.robot_name = ''

        rospy.loginfo('[kb_interface] Creating a knowledge base update client')
        try:
            rospy.wait_for_service('/rosplan_knowledge_base/update', 5.)
            self.knowledge_update_client = rospy.ServiceProxy('/rosplan_knowledge_base/update',
                                                              rosplan_srvs.KnowledgeUpdateService)
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn('The service /rosplan_knowledge_base/state/propositions does not appear to exist.\n' +
                          'Please spawn rosplan_knowledge_base/knowledgeBase')

        rospy.loginfo('[kb_interface] Creating a knowledge base query client')
        try:
            rospy.wait_for_service('/rosplan_knowledge_base/state/propositions', 5.)
            self.attribute_retrieval_client = rospy.ServiceProxy('/rosplan_knowledge_base/state/propositions',
                                                                rosplan_srvs.GetAttributeService)
            request = rosplan_srvs.GetAttributeServiceRequest()
            request.predicate_name = 'robot_name'
            result = self.attribute_retrieval_client(request)
            for item in result.attributes:
                for param in item.values:
                    if param.key == 'bot':
                        self.robot_name = param.value
                        break
                break
        except (rospy.ServiceException, rospy.ROSException) as exc:
            rospy.logwarn('The service /rosplan_knowledge_base/state/propositions does not appear to exist.\n' +
                          'Please spawn rosplan_knowledge_base/knowledgeBase')

        rospy.loginfo('[kb_interface] Creating a message store client')
        try:
            self.msg_store_client = MessageStoreProxy()
        except:
            print('Could not create a mongodb_store proxy.\n' +
                  'Please spawn mongodb_store/message_store_node.py')


    ############################################################################
    #--------------------------- Symbolic knowledge ---------------------------#
    ############################################################################
    def get_all_attributes(self, predicate_name):
        '''Returns a list of all instances of the given predicate in the knowledge base.

        @param predicate_name -- string representing the name of a predicate

        '''
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = predicate_name
        result = self.attribute_retrieval_client(request)
        return result.attributes

    def update_kb(self, facts_to_add, facts_to_remove):
        '''Updates the knowledge base by adding and removing the facts
        from the given lists. In both input lists, each entry is a tuple of the form
        (predicate, [(variable, value), ...]), where "predicate" is the predicate name
        and the (variable, value) tuples are the variable values.

        Keyword arguments:
        @param facts_to_add -- a list of facts to add to the knowledge base
        @param facts_to_remove -- a list of facts to remove from the knowledge base

        '''
        rospy.loginfo('[kb_interface] Inserting facts into the knowledge base')
        self.insert_facts(facts_to_add)

        rospy.loginfo('[kb_interface] Removing facts from the knowledge base')
        self.remove_facts(facts_to_remove)

    def insert_facts(self, fact_list):
        '''Inserts the facts in the given list into the knowledge base.

        Keyword arguments:
        @param fact_list -- a list in which each entry is a tuple of the form
                            (predicate, [(variable, value), ...]), where
                            "predicate" is the predicate name and the
                            (variable, value) tuples are the variable values

        '''
        self.__update_kb(KnowledgeUpdateTypes.INSERT, fact_list)

    def remove_facts(self, fact_list):
        '''Removes the facts in the given list from the knowledge base.

        Keyword arguments:
        @param fact_list -- a list in which each entry is a tuple of the form
                            (predicate, [(variable, value), ...]), where
                            "predicate" is the predicate name and the
                            (variable, value) tuples are the variable values

        '''
        self.__update_kb(KnowledgeUpdateTypes.REMOVE, fact_list)

    def __update_kb(self, update_type, fact_list):
        '''Updates the knowledge base with the facts in the given list.

        Keyword arguments:
        @param update_type -- a KnowledgeUpdateTypes constant describing the update type
                              (add or remove facts)
        @param fact_list -- a list in which each entry is a tuple of the form
                            (predicate, [(variable, value), ...]), where
                            "predicate" is the predicate name and the
                            (variable, value) tuples are the variable values

        '''
        try:
            for predicate, var_data in fact_list:
                # we set up the knowledge update request with the given attribute data
                request = rosplan_srvs.KnowledgeUpdateServiceRequest()
                request.update_type = update_type
                request.knowledge.knowledge_type = 1
                request.knowledge.attribute_name = predicate
                for var_name, var_value in var_data:
                    arg_msg = diag_msgs.KeyValue()
                    arg_msg.key = var_name
                    arg_msg.value = var_value
                    request.knowledge.values.append(arg_msg)

                # we modify the fact into the knowledge base
                self.knowledge_update_client(request)
        except Exception as exc:
            rospy.logerr('[kb_interface] %s', str(exc))
            raise KBException('The knowledge base could not be updated')


    ############################################################################
    #------------------------------ Data storage ------------------------------#
    ############################################################################
    def insert_objects(self, object_list):
        '''Inserts the objects in the given list into the knowledge base.

        Keyword arguments:
        @param object_list -- a list of (obj_name, obj) tuples

        '''
        for obj_name, obj in object_list:
            self.msg_store_client.insert_named(obj_name, obj)

    def insert_obj_instance(self, obj_name, obj):
        '''Inserts a named object instance into the knowledge base.

        Keyword arguments:
        @param obj_name -- name of the object instance
        @param obj -- object to insert

        '''
        self.msg_store_client.insert_named(obj_name, obj)

    def update_objects(self, object_list):
        '''Updates the knowledge about the objects in the given list.

        Keyword arguments:
        @param object_list -- a list of (obj_name, obj) tuples

        '''
        for obj_name, obj in object_list:
            self.msg_store_client.update_named(obj_name, obj)

    def update_obj_instance(self, obj_name, obj):
        '''Updates the knowledge about the given named object.

        Keyword arguments:
        @param obj_name -- name of the object instance
        @param obj -- object to insert

        '''
        self.msg_store_client.update_named(obj_name, obj)

    def remove_objects(self, object_list):
        '''Removes the objects in the given list from the knowledge base.

        Keyword arguments:
        @param object_list -- a list of (obj_name, obj_type) tuples

        '''
        for obj_name, obj_type in object_list:
            self.msg_store_client.delete_named(obj_name, obj_type)

    def remove_obj_instance(self, obj_name, obj_type):
        '''Removes a named object instance from the knowledge base.

        Keyword arguments:
        @param obj_name -- name of the object instance
        @param obj_type -- instance type (the value of the object's "_type" field)

        '''
        self.msg_store_client.delete_named(obj_name, obj_type)

    def get_objects(self, object_names, obj_type):
        '''Returns a list of named object instances from the knowledge base.

        @param object_names -- a list of strings representing object names
        @param obj_type -- type of the objects to be retrieved

        '''
        objects = list()
        for obj_name in object_names:
            obj = self.get_obj_instance(obj_name, obj_type)
            if obj is not None:
                objects.append(obj)
        return objects

    def get_obj_instance(self, obj_name, obj_type):
        '''Retrieves a named object instance from the knowledge base.

        Keyword arguments:
        @param obj_name -- name of the object instance
        @param obj_type -- instance type (the value of the object's "_type" field)

        '''
        try:
            obj = self.msg_store_client.query_named(obj_name, obj_type)[0]
            return obj
        except:
            rospy.logerr('[kb_interface] Error retriving knowledge about %s', obj_name)
            return None

    def save_current_state(self, sm_id, state_name):
        '''Saves the state of the given state machine as a
        "current_state" message store entry.
        '''
        execution_state_msg = ExecutionState()
        execution_state_msg.stamp = rospy.Time.now()
        execution_state_msg.state_machine = sm_id
        execution_state_msg.state = state_name
        try:
            self.msg_store_client.insert_named('current_state', execution_state_msg)
        except:
            print('Error while saving current state')

class KBException(Exception):
    '''A custom knowledge base exception.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self, message):
        super(KBException, self).__init__(message)
        self.message = message

    def __str__(self):
        return self.message

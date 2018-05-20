import rdflib

class KnowledgeBaseManager(object):
    '''Defines an interface for interacting with a domestic knowledge base.
    '''
    def __init__(self, ontology_file):
        self.knowledge_graph = rdflib.Graph()
        self.knowledge_graph.load(ontology_file)

    def is_instance_of(self, obj_name, class_name):
        '''Checks whether 'obj_name' is an instance of 'class_name'.

        Keyword arguments:
        obj_name -- string representing the name of an object
        class_name -- string representing the name of a class

        '''
        pass

    def get_instances_of(self, class_name):
        '''Returns a list of names of all instances belonging to 'class_name'.

        Keyword arguments:
        class_name -- string representing the name of a class

        '''
        pass

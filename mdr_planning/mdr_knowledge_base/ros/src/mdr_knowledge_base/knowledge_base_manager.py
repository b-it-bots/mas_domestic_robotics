import rdflib

class KnowledgeBaseManager(object):
    '''Defines an interface for interacting with an OWL knowledge base
    '''
    def __init__(self, ontology_file, class_prefix):
        self.knowledge_graph = rdflib.Graph()
        self.knowledge_graph.load(ontology_file)
        self.class_prefix = class_prefix

    def is_instance_of(self, obj_name, class_name):
        '''Checks whether 'obj_name' is an instance of 'class_name'

        Keyword arguments:
        obj_name -- string representing the name of an object
        class_name -- string representing the name of a class

        '''
        pass

    def get_instances_of(self, class_name):
        '''Returns a list of names of all instances belonging to 'class_name'

        Keyword arguments:
        class_name -- string representing the name of a class

        '''
        rdf_class = self.class_prefix + ':' + class_name
        query_result = self.knowledge_graph.query('SELECT ?instance ' +
                                                  'WHERE {?instance rdf:type ' + rdf_class + '}')
        instances = list()
        for row in query_result:
            instance = row[0]
            instance_name = instance[instance.rfind('/')+1:]
            instances.append(instance_name)
        return instances

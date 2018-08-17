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

    def get_subclasses_of(self, class_name):
        '''Returns a list of all subclasses of 'class_name'

        Keyword arguments:
        class_name -- string representing the name of a class

        '''
        rdf_class = self.class_prefix + ':' + class_name
        rdf_class_uri = rdflib.URIRef(rdf_class)
        query_result = self.knowledge_graph.transitive_subjects(rdflib.RDFS.subClassOf,
                                                                rdf_class_uri)
        subclasses = list()
        for subclass_uri in query_result:
            subclass = str(subclass_uri)
            subclass = subclass[subclass.find(':')+1:]
            subclasses.append(subclass)
        return subclasses

    def get_parent_classes_of(self, class_name):
        '''Returns a list of all parent classes of 'class_name'

        Keyword arguments:
        class_name -- string representing the name of a class

        '''
        rdf_class = self.class_prefix + ':' + class_name
        rdf_class_uri = rdflib.URIRef(rdf_class)
        query_result = self.knowledge_graph.transitive_objects(rdf_class_uri,
                                                               rdflib.RDFS.subClassOf)
        parent_classes = list()
        for class_uri in query_result:
            parent_class = str(class_uri)
            parent_class = parent_class[parent_class.find(':')+1:]
            parent_classes.append(parent_class)
        return parent_classes

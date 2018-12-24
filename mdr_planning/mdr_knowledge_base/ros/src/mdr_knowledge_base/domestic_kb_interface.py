'''Module defining interfaces for knowledge base interactions
specific to domestic applications.
'''

from mdr_knowledge_base.knowledge_base_interface import KnowledgeBaseInterface

class DomesticKBInterface(KnowledgeBaseInterface):
    '''Defines an interface for performing knowledge base operations common
    for a domestic application.

    @author Alex Mitrevski
    @contact aleksandar.mitrevski@h-brs.de

    '''
    def __init__(self):
        super(DomesticKBInterface, self).__init__()

    ############################################################################
    #--------------------------- Symbolic knowledge ---------------------------#
    ############################################################################
    def get_surface_object_names(self, surface_name):
        '''Returns a list of names of all objects on the given surface.

        Keyword arguments:
        @param surface_name -- string representing the name of a surface

        '''
        surface_objects = list()
        on_instances = self.get_all_attributes('on')
        for item in on_instances:
            object_on_desired_surface = False
            object_name = ''
            if not item.is_negative:
                for param in item.values:
                    if param.key == 'plane' and param.value == surface_name:
                        object_on_desired_surface = True
                    elif param.key == 'obj':
                        object_name = param.value
            if object_on_desired_surface:
                surface_objects.append(object_name)
        return surface_objects

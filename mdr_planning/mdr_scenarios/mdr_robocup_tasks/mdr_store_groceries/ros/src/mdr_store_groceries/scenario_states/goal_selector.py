import numpy as np

from mas_execution_manager.scenario_state_base import ScenarioStateBase
from mdr_rosplan_interface.planner_interface import PlannerInterface

class StoreGroceriesGoalSelector(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'store_groceries_goal_selector',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'store_groceries_goal_selector')
        self.number_of_shelves = kwargs.get('number_of_shelves', -1)
        self.planner_interface = PlannerInterface()

    def execute(self, userdata):
        # we check if the table and all shelves have been explored.
        # if they have been, we call the planner with the goal of storing
        # the groceries; otherwise, the goal is to scan them
        if not self.exploration_done():
            self.say('I need to check out the table and shelves')

            storing_goals = self.get_storing_groceries_goals()
            self.planner_interface.remove_plan_goals(storing_goals)

            scanning_goals = self.get_scanning_goals()
            self.planner_interface.add_plan_goals(scanning_goals)
        else:
            self.say('I will now store the groceries')

            scanning_goals = self.get_scanning_goals()
            self.planner_interface.remove_plan_goals(scanning_goals)

            storing_goals = self.get_storing_groceries_goals()
            self.planner_interface.add_plan_goals(storing_goals)

        plan_generated = self.planner_interface.plan()
        if plan_generated:
            plan_executed = self.planner_interface.start_plan_dispatch()

        if plan_executed:
            return 'succeeded'
        return 'failed'

    def exploration_done(self):
        '''Returns True if the table and all shelves (as many as expected based on
        self.number_of_shelves) have been explored; returns False otherwise.
        '''
        explored_table = False
        explored_shelf_count = 0

        explored_attributes = self.planner_interface.kb_interface.get_all_attributes('explored')
        for attribute in explored_attributes:
            for val in attribute.values:
                if val.key == 'plane':
                    if val.value.find('table') != -1:
                        explored_table = True
                    elif val.value.find('shelf') != -1:
                        explored_shelf_count += 1

        return explored_table and explored_shelf_count == self.number_of_shelves

    def get_scanning_goals(self):
        '''Returns a list of planning goals for scanning the table and shelves.
        '''
        goals = [('explored', [('plane', 'table')])]
        for i in range(self.number_of_shelves):
            goals.append(('explored', [('plane', 'shelf{0}'.format(i+1))]))
        return goals

    def get_storing_groceries_goals(self):
        '''Adds a list of "stored_on" predicates to the knowledge base that indicate
        which category of objects is stored on which shelf and returns a list of
        "on" predicates indicating where each object should be stored.
        '''
        obj_category_map = self.planner_interface.kb_interface.get_obj_category_map()
        category_storage_locations = self.get_category_storing_locations(obj_category_map)

        # we add predicates about the storage locations of objects to the knowledge base
        storage_fact_list = []
        for category, location in category_storage_locations.items():
            storage_fact_list.append(('stored_on', [('class', category),
                                                    ('plane', location)]))
        self.planner_interface.kb_interface.insert_facts(storage_fact_list)

        # we create a list of storage goals - an "on" predicate per object that needs to be stored
        goals = []
        for obj, obj_category in obj_category_map.items():
            goals.append(('on', [('obj', obj),
                                 ('plane', category_storage_locations[obj_category])]))
        return goals

    def get_category_storing_locations(self, obj_category_map):
        '''Returns a dictionary in which each key is an object category and the
        value specifies which shelf that object category is stored on.
        This information is obtained by retriving the categories of
        the objects seen on the individual shelves.

        The storage locations of object categories that haven't been observed
        on the shelves are assigned randomly.
        '''
        # we keep track of the categories whose locations have been assigned
        # so that we can check if there are categories for which the storage
        # location is not known; if there are such categories, we select a
        # storage location from the available shelves at random
        assigned_categories = set()

        category_storage_locations = dict()
        for i in range(self.number_of_shelves):
            shelf_name = 'shelf{0}'.format(i+1)
            shelf_object_names = self.planner_interface.kb_interface.get_surface_object_names(shelf_name)
            for obj in shelf_object_names:
                # TODO: what to do if the recognition fails and some category
                # of objects seems to be stored on multiple shelves?
                category_storage_locations[obj_category_map[obj]] = shelf_name
                assigned_categories.add(obj_category_map[obj])

        unassigned_categories = set([cat for obj, cat in obj_category_map.items()]) - assigned_categories
        if unassigned_categories:
            print('Storage locations of {0} unknown'.format(unassigned_categories))
            for category in unassigned_categories:
                shelf_idx = np.random.randint(1, self.number_of_shelves+1)
                shelf_name = 'shelf{0}'.format(shelf_idx)
                print('Setting storage location of "{0}" to "{1}"'.format(category,
                                                                          shelf_name))
                category_storage_locations[category] = shelf_name

        return category_storage_locations

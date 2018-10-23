import time
import numpy as np
import rospy
from std_msgs.msg import String

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
import diagnostic_msgs.msg as diag_msgs

from mcr_perception_msgs.msg import Object
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class Place(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'place',
                                   save_sm_state=save_sm_state,
                                   input_keys=['grasped_object'],
                                   outcomes=['pick_new_object', 'finished',
                                             'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', 'mdr_store_groceries')
        self.state_name = kwargs.get('state_name', 'place')
        self.timeout = kwargs.get('timeout', 120.)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        grasped_object = userdata.grasped_object
        grasped_obj_category = self.get_object_category(grasped_object)
        surface_name = self.choose_placing_surface(grasped_object, grasped_obj_category)
        dispatch_msg = self.get_dispatch_msg(grasped_object, surface_name)
        rospy.loginfo('Placing %s on %s' % (grasped_object, surface_name))
        self.say('Placing ' + grasped_object + ' on ' + surface_name)
        self.action_dispatch_pub.publish(dispatch_msg)

        self.executing = True
        self.succeeded = False
        start_time = time.time()
        duration = 0.
        while self.executing and duration < self.timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.succeeded:
            rospy.loginfo('Object placed successfully')
            self.say('Successfully placed ' + grasped_object)
            if self.surface_empty(surface_name='table'):
                return 'finished'
            return 'pick_new_object'

        rospy.loginfo('Could not place object %s' % grasped_object)
        self.say('Could not place ' + obj_to_grasp)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to place object %s' % grasped_object)
            self.say('Aborting operation')
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to place object %s' % grasped_object)
        self.retry_count += 1
        return 'failed'

    def get_object_category(self, obj_name):
        try:
            obj = self.msg_store_client.query_named(obj_name, Object._type)[0]
            return obj.category
        except:
            rospy.logerr('Error retriving knowledge about %s', obj_name)
            return ''

    def choose_placing_surface(self, obj_name, obj_category):
        obj_category_map = self.get_obj_category_map()
        surface_category_counts = self.get_surface_category_counts(obj_category_map)
        grasped_object_category = obj_category_map[obj_name]
        placing_surface = self.get_best_placing_surface(obj_category,
                                                        surface_category_counts)
        return placing_surface

    def get_obj_category_map(self):
        '''Returns a dictionary of objects and object categories in which
        each key represents an object and the value is its category
        '''
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'object_category'
        category_result = self.attribute_fetching_client(request)

        obj_category_dict = dict()
        for item in category_result.attributes:
            obj_name = ''
            obj_category = ''
            for param in item.values:
                if param.key == 'obj':
                    obj_name = param.value
                elif param.key == 'cat':
                    obj_category = param.value
            obj_category_dict[obj_name] = obj_category
        return obj_category_dict

    def get_surface_category_counts(self, obj_category_map):
        '''Returns a dictionary of surfaces and object category counts in which
        each key represents a surface in the environment and the value
        is a dictionary of object counts for each category
        '''
        surface_category_counts = dict()

        # we take all explored surfaces and populate 'surface_category_counts'
        # with an empty dictionary for each surface; each such dictionary
        # will store the object category count for the respective surface
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'explored'
        explored_result = self.attribute_fetching_client(request)
        for item in explored_result.attributes:
            surface_name = ''
            for param in item.values:
                if param.key == 'plane':
                    surface_name = param.value
                    # we don't want to place items on the table, so we
                    # don't consider the table as a placing surface
                    if surface_name not in surface_category_counts and surface_name != 'table':
                        surface_category_counts[surface_name] = dict()

        # we collect a dictionary of object category counts for each surface
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'on'
        on_result = self.attribute_fetching_client(request)
        for item in on_result.attributes:
            obj_name = ''
            obj_surface = ''
            for param in item.values:
                if param.key == 'obj':
                    obj_name = param.value
                elif param.key == 'plane':
                    obj_surface = param.value

            if obj_surface != 'table':
                obj_category = obj_category_map[obj_name]
                if obj_category not in surface_category_counts[obj_surface]:
                    surface_category_counts[obj_surface][obj_category] = 1
                else:
                    surface_category_counts[obj_surface][obj_category] += 1
        return surface_category_counts

    def get_best_placing_surface(self, obj_category, surface_category_counts):
        '''If none of the surfaces contain objects whose category is
        'obj_category', returns the name of a random surface; otherwise,
        returns the name of the surface that has the largest number
        of objects of the category 'obj_category'.
        '''
        surfaces = list()
        obj_surface_category_counts = list()
        for surface, category_counts in surface_category_counts.items():
            surfaces.append(surface)
            if obj_category in category_counts:
                obj_surface_category_counts.append(category_counts[obj_category])
            else:
                obj_surface_category_counts.append(0)

        surface_idx = -1
        if np.count_nonzero(obj_surface_category_counts) != 0:
            surface_idx = np.argmax(obj_surface_category_counts)
        else:
            surface_idx = np.random.randint(0, len(surfaces))
        return surfaces[surface_idx]

    def surface_empty(self, surface_name='table'):
        no_objects_on_surface = True
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'on'
        result = self.attribute_fetching_client(request)
        for item in result.attributes:
            object_on_desired_surface = False
            if not item.is_negative:
                for param in item.values:
                    if param.key == 'plane' and param.value == surface_name:
                        object_on_desired_surface = True
            if object_on_desired_surface:
                no_objects_on_surface = False
                break
        return no_objects_on_surface

    def get_dispatch_msg(self, obj_name, plane_name):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = self.action_name

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'bot'
        arg_msg.value = self.robot_name
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'obj'
        arg_msg.value = obj_name
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'plane'
        arg_msg.value = plane_name
        dispatch_msg.parameters.append(arg_msg)

        return dispatch_msg

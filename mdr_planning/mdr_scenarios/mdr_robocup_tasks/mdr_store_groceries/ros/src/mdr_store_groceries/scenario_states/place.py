import time
import numpy as np
import rospy

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
import diagnostic_msgs.msg as diag_msgs

from mcr_perception_msgs.msg import Object
from mdr_store_groceries.scenario_states.scenario_state_base import ScenarioStateBase

class Place(ScenarioStateBase):
    def __init__(self, **kwargs):
        ScenarioStateBase.__init__(self, 'place',
                                   input_keys=['grasped_object'],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)

    def execute(self, userdata):
        grasped_object = userdata.grasped_object
        grasped_obj_category = self.get_object_category(grasped_object)
        surface_name = self.choose_placing_surface(grasped_object, grasped_obj_category)
        dispatch_msg = self.get_dispatch_msg(grasped_object, surface_name)
        rospy.loginfo('Placing %s on %s' % (grasped_object, surface_name))
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
            return 'succeeded'

        rospy.loginfo('Could not place object %s' % grasped_object)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to place object %s' % grasped_object)
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
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'on'
        on_result = self.attribute_fetching_client(request)

        surface_category_counts = dict()
        for item in on_result.attributes:
            obj_name = ''
            obj_surface = ''
            for param in item.values:
                if param.key == 'obj':
                    obj_name = param.value
                elif param.key == 'plane':
                    obj_surface = param.value
                    if obj_surface not in surface_category_counts:
                        surface_category_counts[obj_surface] = dict()

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

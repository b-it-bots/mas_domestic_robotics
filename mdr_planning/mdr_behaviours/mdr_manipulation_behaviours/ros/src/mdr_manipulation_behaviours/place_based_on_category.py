import time
import numpy as np
import rospy

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs

from mas_execution_manager.scenario_state_base import ScenarioStateBase

class PlaceBasedOnCategory(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'place',
                                   save_sm_state=save_sm_state,
                                   input_keys=['grasped_object'],
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'place_based_on_category')
        self.placing_surface_prefix = kwargs.get('placing_surface_prefix', '')
        self.timeout = kwargs.get('timeout', 120.)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        grasped_object = userdata.grasped_object
        surface_name = self.choose_placing_surface(grasped_object)
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
            return 'succeeded'

        rospy.loginfo('Could not place object %s' % grasped_object)
        self.say('Could not place ' + grasped_object)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to place object %s' % grasped_object)
            self.say('Aborting operation')
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to place object %s' % grasped_object)
        self.retry_count += 1
        return 'failed'

    def choose_placing_surface(self, obj_name):
        obj_category_map = self.kb_interface.get_obj_category_map()
        surface_category_counts = self.kb_interface.get_surface_category_counts(self.placing_surface_prefix,
                                                                                obj_category_map)
        grasped_object_category = obj_category_map[obj_name]
        placing_surface = self.get_best_placing_surface(grasped_object_category,
                                                        surface_category_counts)
        return placing_surface

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

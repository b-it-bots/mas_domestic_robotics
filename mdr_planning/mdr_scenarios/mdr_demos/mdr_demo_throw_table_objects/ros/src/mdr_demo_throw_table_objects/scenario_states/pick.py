import time
import numpy as np
import rospy
from std_msgs.msg import String
from tf import TransformListener

from geometry_msgs.msg import PointStamped

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
import diagnostic_msgs.msg as diag_msgs

from mcr_perception_msgs.msg import Object
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class Pick(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'pickup',
                                   save_sm_state=save_sm_state,
                                   output_keys=['grasped_object'],
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying',
                                             'find_objects_before_picking'])
        self.sm_id = kwargs.get('sm_id', 'mdr_demo_throw_table_objects')
        self.state_name = kwargs.get('state_name', 'pick')
        self.timeout = kwargs.get('timeout', 120.)
        self.tf_listener = TransformListener()

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        surface_objects = self.get_surface_objects(surface_prefix='table')
        object_poses = self.get_object_poses(surface_objects)
        surface, obj_to_grasp_idx = self.select_object_for_grasping(object_poses)
        obj_to_grasp = ''
        if obj_to_grasp_idx != -1:
            obj_to_grasp = surface_objects[surface][obj_to_grasp_idx]
        else:
            rospy.logerr('Could not find an object to grasp')
            self.say('Could not find an object to grasp')
            return 'find_objects_before_picking'

        dispatch_msg = self.get_dispatch_msg(obj_to_grasp, surface)
        rospy.loginfo('Picking %s from %s' % (obj_to_grasp, surface))
        self.say('Picking ' + obj_to_grasp + ' from ' + surface)
        self.action_dispatch_pub.publish(dispatch_msg)

        self.executing = True
        self.succeeded = False
        start_time = time.time()
        duration = 0.
        while self.executing and duration < self.timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.succeeded:
            rospy.loginfo('%s grasped successfully' % obj_to_grasp)
            self.say('Successfully grasped ' + obj_to_grasp)
            userdata.grasped_object = obj_to_grasp
            return 'succeeded'

        rospy.loginfo('Could not grasp %s' % obj_to_grasp)
        self.say('Could not grasp ' + obj_to_grasp)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to grasp %s' % obj_to_grasp)
            self.say('Aborting operation')
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to grasp %s' % obj_to_grasp)
        self.retry_count += 1
        return 'failed'

    def get_surface_objects(self, surface_prefix='table'):
        surface_objects = dict()
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'on'
        result = self.attribute_fetching_client(request)
        for item in result.attributes:
            object_on_desired_surface = False
            object_name = ''
            surface_name = ''
            if not item.is_negative:
                for param in item.values:
                    if param.key == 'plane' and param.value.find(surface_prefix) != -1:
                        object_on_desired_surface = True
                        surface_name = param.value
                        if surface_name not in surface_objects:
                            surface_objects[surface_name] = list()
                    elif param.key == 'obj':
                        object_name = param.value
            if object_on_desired_surface:
                surface_objects[surface_name].append(object_name)
        return surface_objects

    def get_object_poses(self, surface_objects):
        object_poses = dict()
        for surface, objects in surface_objects.items():
            object_poses[surface] = list()
            for obj_name in objects:
                try:
                    obj = self.msg_store_client.query_named(obj_name, Object._type)[0]
                    object_poses[surface].append(obj.pose)
                except:
                    rospy.logerr('Error retriving knowledge about %s', obj_name)
                    pass
        return object_poses

    def select_object_for_grasping(self, object_poses):
        '''Returns the index of the object whose distance is closest to the robot
        '''
        object_surfaces = list()
        distances = dict()
        robot_position = np.zeros(3)
        for surface, poses in object_poses.items():
            object_surfaces.append(surface)
            distances[surface] = list()
            for pose in poses:
                base_link_pose = self.tf_listener.transformPose('base_link', pose)
                distances[surface].append(self.distance(robot_position, np.array([base_link_pose.pose.position.x,
                                                                                  base_link_pose.pose.position.y,
                                                                                  base_link_pose.pose.position.z])))

        min_dist = 1e100
        min_dist_obj_idx = -1
        obj_surface = ''
        for surface, distance_list in distances.items():
            if distance_list:
                surface_min_dist = np.min(distance_list)
                if surface_min_dist < min_dist:
                    min_dist = surface_min_dist
                    min_dist_obj_idx = np.argmin(distance_list)
                    obj_surface = surface
        return obj_surface, min_dist_obj_idx

    def get_robot_pose(self, map_frame='map', base_link_frame='base_link'):
        latest_tf_time = self.tf_listener.getLatestCommonTime(base_link_frame, map_frame)
        position, quat_orientation = self.tf_listener.lookupTransform(base_link_frame,
                                                                      map_frame,
                                                                      latest_tf_time)
        return position, quat_orientation

    def distance(self, point1, point2):
        return np.linalg.norm(np.array(point1) - np.array(point2))

    def get_dispatch_msg(self, obj_name, surface_name):
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
        arg_msg.value = surface_name
        dispatch_msg.parameters.append(arg_msg)

        return dispatch_msg

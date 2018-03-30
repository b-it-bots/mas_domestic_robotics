import time
import numpy as np
import rospy
from tf import TransformListener

from geometry_msgs.msg import PointStamped

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
import diagnostic_msgs.msg as diag_msgs

from mcr_perception_msgs.msg import Object
from mdr_store_groceries.scenario_states.scenario_state_base import ScenarioStateBase

class Pick(ScenarioStateBase):
    def __init__(self, **kwargs):
        ScenarioStateBase.__init__(self, 'pickup',
                                   output_keys=['grasped_object'],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.tf_listener = TransformListener()

    def execute(self, userdata):
        surface_objects = self.get_surface_objects()
        object_poses = self.get_object_poses(surface_objects)
        obj_to_grasp_idx = self.select_object_for_grasping(object_poses)
        obj_to_grasp = surface_objects[obj_to_grasp_idx]

        dispatch_msg = self.get_dispatch_msg(obj_to_grasp, 'table')
        rospy.loginfo('Picking %s from the table' % obj_to_grasp)
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
            userdata.grasped_object = obj_to_grasp
            return 'succeeded'

        rospy.loginfo('Could not grasp %s' % obj_to_grasp)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to grasp %s' % obj_to_grasp)
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to grasp %s' % obj_to_grasp)
        self.retry_count += 1
        return 'failed'

    def get_surface_objects(self, surface_name='table'):
        surface_objects = list()
        request = rosplan_srvs.GetAttributeServiceRequest()
        request.predicate_name = 'on'
        result = self.attribute_fetching_client(request)
        for item in result.attributes:
            if not item.is_negative:
                for param in item.values:
                    if (param.key == 'bot' and param.value != self.robot_name) \
                    or (param.key == 'plane' and param.value != surface_name):
                        break

                    if param.key == 'obj':
                        surface_objects.append(param.value)
        return surface_objects

    def get_object_poses(self, surface_objects):
        object_poses = list()
        for obj_name in surface_objects:
            try:
                obj = self.msg_store_client.query_named(obj_name, Object._type)
                object_poses.append(obj.pose)
            except:
                pass
        return object_poses

    def select_object_for_grasping(self, object_poses):
        '''Returns the index of the object whose distance is closest to the robot
        '''
        distances = list()
        robot_position, robot_orientation = self.get_robot_pose(map_frame='map',
                                                                base_link_frame='base_link')
        for pose in object_poses:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = pose.header.frame_id
            point_stamped.header.stamp = rospy.Time(0)
            point_stamped.point.x = pose.pose.position.x
            point_stamped.point.y = pose.pose.position.y
            point_stamped.point.z = pose.pose.position.z
            point_map_pos = self.tf_listener.transformPoint('map', point_stamped)
            distances.append(self.distance(robot_position, np.array([point_map_pos.point.x,
                                                                     point_map_pos.point.y,
                                                                     point_map_pos.point.z])))

        min_dist_obj_idx = np.argmin(distances)
        return min_dist_obj_idx

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

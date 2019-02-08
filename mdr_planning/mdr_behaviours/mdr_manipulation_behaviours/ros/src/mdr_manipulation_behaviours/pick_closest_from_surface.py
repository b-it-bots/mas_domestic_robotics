import time
import numpy as np
import rospy
from tf import TransformListener

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs

from mcr_perception_msgs.msg import Object
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class PickClosestFromSurface(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'pickup',
                                   save_sm_state=save_sm_state,
                                   output_keys=['grasped_object'],
                                   outcomes=['succeeded', 'failed',
                                             'failed_after_retrying',
                                             'find_objects_before_picking'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'pick')
        self.timeout = kwargs.get('timeout', 120.)
        self.picking_surface_prefix = kwargs.get('picking_surface_prefix', 120.)
        self.tf_listener = TransformListener()

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        surface_objects = self.kb_interface.get_surface_object_map(self.picking_surface_prefix)
        object_poses = self.kb_interface.get_surface_object_pose_map(surface_objects, Object._type)
        surface, obj_to_grasp = self.select_object_for_grasping(object_poses)
        if not obj_to_grasp:
            rospy.logerr('Could not find an object to grasp')
            self.say('Could not find an object to grasp')
            if self.retry_count == self.number_of_retries:
                rospy.logerr('No object could be found; giving up')
                self.say('Could not find an object to grasp; giving up')
                return 'failed_after_retrying'
            self.retry_count += 1
            return 'find_objects_before_picking'

        self.retry_count = 0
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

    def select_object_for_grasping(self, surface_object_poses):
        '''Returns the index of the object whose distance is closest to the robot
        '''
        robot_position = np.zeros(3)
        min_dist = 1e100
        min_dist_obj = ''
        obj_surface = ''
        for surface, object_poses in surface_object_poses.items():
            for obj, pose in object_poses.items():
                base_link_pose = self.tf_listener.transformPose('base_link', pose)
                dist = self.distance(robot_position, np.array([base_link_pose.pose.position.x,
                                                               base_link_pose.pose.position.y,
                                                               base_link_pose.pose.position.z]))
                if dist < min_dist:
                    min_dist = dist
                    min_dist_obj = obj
                    obj_surface = surface
        return obj_surface, min_dist_obj

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

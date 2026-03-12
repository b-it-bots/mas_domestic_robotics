#!/usr/bin/env python3
"""
PlaceObject smach state - Place an object at a location.

Uses mdr_place_action to place objects.
Uses perceived_planes from PerceiveTable to get the placing surface.
"""

import rospy
import smach
import actionlib
import tf

from mdr_place_action.msg import PlaceAction, PlaceGoal
from geometry_msgs.msg import PoseStamped


class PlaceObject(smach.State):
    """
    Place an object on a perceived surface.
    
    Uses perceived_planes userdata to get the table surface pose.
    
    Outcomes:
        succeeded              - object placed successfully
        failed                 - place failed
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self,
                 place_server='place_server',
                 placing_surface_prefix='table',
                 z_offset=0.05,
                 y_offset=0.1,
                 timeout=120.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=['perceived_planes', 'grasped_object_height']
        )
        self.placing_surface_prefix = placing_surface_prefix
        self.z_offset = z_offset
        self.y_offset = y_offset
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

        rospy.loginfo('[PlaceObject] Connecting to %s...', place_server)
        self.place_client = actionlib.SimpleActionClient(place_server, PlaceAction)
        self.place_client.wait_for_server()
        rospy.loginfo('[PlaceObject] Connected.')
        
        self.tf_listener = tf.TransformListener()

    def execute(self, userdata):
        # Get the placing surface from perceived planes
        planes = userdata.perceived_planes.planes
        relevant = [p for p in planes if self.placing_surface_prefix in p.name]
        
        if not relevant:
            rospy.logwarn('[PlaceObject] No "%s" surface found', self.placing_surface_prefix)
            return self._retry()
        
        # Use the first matching plane
        plane = relevant[0]
        
        # Transform plane point to base_link
        plane_ps = PoseStamped()
        plane_ps.header = plane.header
        plane_ps.pose.position.x = plane.plane_point.x
        plane_ps.pose.position.y = plane.plane_point.y
        plane_ps.pose.position.z = plane.plane_point.z
        plane_ps.pose.orientation.w = 1.0
        
        try:
            self.tf_listener.waitForTransform('base_link', plane_ps.header.frame_id, 
                                               rospy.Time(0), rospy.Duration(2.0))
            plane_bl = self.tf_listener.transformPose('base_link', plane_ps)
        except Exception as e:
            rospy.logerr('[PlaceObject] TF transform failed: %s', e)
            return self._retry()
        
        # Get object height from userdata (set by PickObject)
        obj_height = getattr(userdata, 'grasped_object_height', 0.05)
        if obj_height is None:
            obj_height = 0.05
        
        # Calculate z: table surface + half object height + clearance
        # z_offset ensures the object bottom clears the surface
        place_z = plane_bl.pose.position.z + (obj_height / 2.0) + self.z_offset
        
        # Build place pose: on the table surface, offset to the side
        goal = PlaceGoal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'base_link'
        goal.pose.header.stamp = rospy.Time.now()
        goal.pose.pose.position.x = plane_bl.pose.position.x
        goal.pose.pose.position.y = plane_bl.pose.position.y + self.y_offset  # Offset to side
        goal.pose.pose.position.z = place_z
        goal.pose.pose.orientation.w = 1.0

        rospy.loginfo('[PlaceObject] Placing at x=%.3f y=%.3f z=%.3f (table_z=%.3f, obj_height=%.3f)',
                      goal.pose.pose.position.x,
                      goal.pose.pose.position.y,
                      goal.pose.pose.position.z,
                      plane_bl.pose.position.z,
                      obj_height)
        
        self.place_client.send_goal(goal)
        
        finished = self.place_client.wait_for_result(rospy.Duration(self.timeout))
        
        if not finished:
            self.place_client.cancel_goal()
            rospy.logerr('[PlaceObject] Timed out')
            return self._retry()

        result = self.place_client.get_result()
        if result and result.success:
            rospy.loginfo('[PlaceObject] Object placed successfully')
            self.retry_count = 0
            return 'succeeded'

        rospy.logwarn('[PlaceObject] Place failed')
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[PlaceObject] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'


class PlaceInContainer(smach.State):
    """
    Place an object in a specific container (dishwasher, trash bin, cabinet).
    
    Outcomes:
        succeeded              - object placed successfully
        failed                 - place failed
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self,
                 container_name,
                 place_server='place_server',
                 timeout=120.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=['grasped_object']
        )
        self.container_name = container_name
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

        rospy.loginfo('[PlaceInContainer] Connecting to %s...', place_server)
        self.place_client = actionlib.SimpleActionClient(place_server, PlaceAction)
        self.place_client.wait_for_server()

    def execute(self, userdata):
        goal = PlaceGoal()
        # Container-specific placement logic would go here
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = rospy.Time.now()
        
        rospy.loginfo('[PlaceInContainer] Placing in %s...', self.container_name)
        self.place_client.send_goal(goal)
        
        finished = self.place_client.wait_for_result(rospy.Duration(self.timeout))
        
        if not finished:
            self.place_client.cancel_goal()
            return self._retry()

        result = self.place_client.get_result()
        if result and result.success:
            rospy.loginfo('[PlaceInContainer] Placed in %s', self.container_name)
            self.retry_count = 0
            return 'succeeded'

        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'

#!/usr/bin/env python3
"""
PlaceObject smach state - Place an object at a location.

Uses mdr_place_action to place objects.
"""

import rospy
import smach
import actionlib

from mdr_place_action.msg import PlaceAction, PlaceGoal
from geometry_msgs.msg import PoseStamped


class PlaceObject(smach.State):
    """
    Place an object at a specified location.
    
    Outcomes:
        succeeded              - object placed successfully
        failed                 - place failed
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self,
                 place_server='place_server',
                 placing_surface='table',
                 timeout=120.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=['place_pose']  # optional: specific pose to place at
        )
        self.placing_surface = placing_surface
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

        rospy.loginfo('[PlaceObject] Connecting to %s...', place_server)
        self.place_client = actionlib.SimpleActionClient(place_server, PlaceAction)
        self.place_client.wait_for_server()
        rospy.loginfo('[PlaceObject] Connected.')

    def execute(self, userdata):
        goal = PlaceGoal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'base_link'
        goal.pose.header.stamp = rospy.Time.now()
        
        # Use provided pose or default position
        if hasattr(userdata, 'place_pose') and userdata.place_pose:
            goal.pose = userdata.place_pose
        else:
            # Default placing pose in front of robot
            goal.pose.pose.position.x = 0.5
            goal.pose.pose.position.y = 0.0
            goal.pose.pose.position.z = 0.8
            goal.pose.pose.orientation.w = 1.0

        rospy.loginfo('[PlaceObject] Placing object on %s...', self.placing_surface)
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

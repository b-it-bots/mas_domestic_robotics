#!/usr/bin/env python3
"""
GazeControl smach states - Control robot head/gaze direction.

Handles looking at people, navigation direction, and objects.
"""

import rospy
import smach
import math

from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool


class LookAtPerson(smach.State):
    """
    Look at a detected person (track their face).
    
    Outcomes:
        succeeded - looking at person
        failed    - could not look at person
    """

    def __init__(self,
                 head_topic='/head_controller/point_head',
                 duration=0.0):
        """
        Args:
            head_topic: Topic to publish head target point
            duration: How long to track (0 = once, >0 = track for duration)
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed'],
            input_keys=['person_pose']  # optional
        )
        self.head_pub = rospy.Publisher(head_topic, PointStamped, queue_size=1)
        self.duration = duration

    def execute(self, userdata):
        rospy.loginfo('[LookAtPerson] Looking at person...')
        
        point = PointStamped()
        point.header.frame_id = 'base_link'
        point.header.stamp = rospy.Time.now()
        
        # Use provided pose or default "in front"
        if hasattr(userdata, 'person_pose') and userdata.person_pose:
            point.point.x = userdata.person_pose.pose.position.x
            point.point.y = userdata.person_pose.pose.position.y
            point.point.z = userdata.person_pose.pose.position.z + 0.3  # Look at face
        else:
            # Default: look forward at face height
            point.point.x = 1.5
            point.point.y = 0.0
            point.point.z = 1.5
        
        self.head_pub.publish(point)
        
        if self.duration > 0:
            rospy.sleep(self.duration)
        
        return 'succeeded'


class LookAtNavigationGoal(smach.State):
    """
    Look in the direction of navigation (forward).
    
    Outcomes:
        succeeded - looking forward
    """

    def __init__(self, head_topic='/head_controller/point_head'):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.head_pub = rospy.Publisher(head_topic, PointStamped, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo('[LookAtNavigationGoal] Looking forward...')
        
        point = PointStamped()
        point.header.frame_id = 'base_link'
        point.header.stamp = rospy.Time.now()
        point.point.x = 3.0
        point.point.y = 0.0
        point.point.z = 1.0
        
        self.head_pub.publish(point)
        return 'succeeded'


class LookAtPoint(smach.State):
    """
    Look at a specific point in space.
    
    Outcomes:
        succeeded - looking at point
    """

    def __init__(self, x=1.0, y=0.0, z=1.0, frame_id='base_link',
                 head_topic='/head_controller/point_head'):
        smach.State.__init__(
            self,
            outcomes=['succeeded'],
            input_keys=['target_point']  # optional override
        )
        self.x = x
        self.y = y
        self.z = z
        self.frame_id = frame_id
        self.head_pub = rospy.Publisher(head_topic, PointStamped, queue_size=1)

    def execute(self, userdata):
        point = PointStamped()
        point.header.frame_id = self.frame_id
        point.header.stamp = rospy.Time.now()
        
        if hasattr(userdata, 'target_point') and userdata.target_point:
            point.point = userdata.target_point
        else:
            point.point.x = self.x
            point.point.y = self.y
            point.point.z = self.z
        
        rospy.loginfo('[LookAtPoint] Looking at (%.2f, %.2f, %.2f)',
                      point.point.x, point.point.y, point.point.z)
        self.head_pub.publish(point)
        return 'succeeded'

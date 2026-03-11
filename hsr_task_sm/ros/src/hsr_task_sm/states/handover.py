#!/usr/bin/env python3
"""
Handover smach states - Object handover with humans.

Handles both receiving objects from humans and giving objects to humans.
"""

import rospy
import smach
import actionlib

from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Empty


class ReceiveObject(smach.State):
    """
    Receive an object from a human through handover.
    
    Outcomes:
        succeeded              - object received
        failed                 - handover failed
        failed_after_retrying  - max retries exhausted
    Output userdata:
        received_object - info about the received object
    """

    def __init__(self,
                 gripper_topic='/gripper/command',
                 timeout=30.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            output_keys=['received_object']
        )
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

    def execute(self, userdata):
        rospy.loginfo('[ReceiveObject] Preparing to receive object...')
        
        # Move arm to receive position
        # (Integrate with actual arm controller)
        
        # Open gripper
        rospy.loginfo('[ReceiveObject] Opening gripper, ready to receive...')
        
        # Wait for object detection in gripper
        rospy.loginfo('[ReceiveObject] Waiting for object...')
        rospy.sleep(3.0)  # Placeholder - use force/torque or vision
        
        # Close gripper
        rospy.loginfo('[ReceiveObject] Object detected, closing gripper...')
        
        # Announce
        rospy.loginfo('[ReceiveObject] Object received!')
        userdata.received_object = {'name': 'bag', 'timestamp': rospy.Time.now()}
        self.retry_count = 0
        return 'succeeded'

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'


class GiveObject(smach.State):
    """
    Give an object to a human through handover.
    
    Outcomes:
        succeeded              - object given
        failed                 - handover failed
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self,
                 timeout=30.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=['object_to_give']
        )
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

    def execute(self, userdata):
        rospy.loginfo('[GiveObject] Preparing to give object...')
        
        # Move arm to handover position
        rospy.loginfo('[GiveObject] Extending arm for handover...')
        
        # Wait for human to grasp
        rospy.loginfo('[GiveObject] Please take the object...')
        rospy.sleep(3.0)  # Placeholder
        
        # Detect when object is taken (force/torque change)
        rospy.loginfo('[GiveObject] Object taken, releasing...')
        
        # Open gripper
        rospy.loginfo('[GiveObject] Object given successfully!')
        self.retry_count = 0
        return 'succeeded'

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'

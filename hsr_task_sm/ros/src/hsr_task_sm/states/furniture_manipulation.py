#!/usr/bin/env python3
"""
Door and furniture manipulation states.

Handles opening/closing doors, dishwashers, and other furniture.
"""

import rospy
import smach
import actionlib


class OpenDoor(smach.State):
    """
    Open a door (entrance door, dishwasher, cabinet).
    
    Outcomes:
        succeeded              - door opened
        failed                 - could not open door
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self,
                 door_type='entrance',
                 timeout=60.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying']
        )
        self.door_type = door_type
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

    def execute(self, userdata):
        rospy.loginfo('[OpenDoor] Opening %s door...', self.door_type)
        
        # Door-specific manipulation logic
        # This would integrate with actual manipulation actions
        
        # Placeholder: simulate door opening
        rospy.sleep(2.0)
        
        rospy.loginfo('[OpenDoor] %s door opened!', self.door_type.capitalize())
        self.retry_count = 0
        return 'succeeded'

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'


class CloseDoor(smach.State):
    """
    Close a door (dishwasher, cabinet).
    
    Outcomes:
        succeeded              - door closed
        failed                 - could not close door
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self,
                 door_type='dishwasher',
                 timeout=60.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying']
        )
        self.door_type = door_type
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

    def execute(self, userdata):
        rospy.loginfo('[CloseDoor] Closing %s door...', self.door_type)
        
        # Placeholder
        rospy.sleep(2.0)
        
        rospy.loginfo('[CloseDoor] %s door closed!', self.door_type.capitalize())
        self.retry_count = 0
        return 'succeeded'

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'failed'


class PullDishwasherRack(smach.State):
    """
    Pull/push the dishwasher rack.
    
    Outcomes:
        succeeded - rack manipulated
        failed    - could not manipulate rack
    """

    def __init__(self, action='pull', timeout=30.0):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed']
        )
        self.action = action
        self.timeout = timeout

    def execute(self, userdata):
        rospy.loginfo('[PullDishwasherRack] %sing dishwasher rack...', 
                      self.action.capitalize())
        
        # Placeholder
        rospy.sleep(2.0)
        
        rospy.loginfo('[PullDishwasherRack] Rack %sed!', self.action)
        return 'succeeded'


class WaitForDoorbell(smach.State):
    """
    Wait for doorbell sound.
    
    Outcomes:
        doorbell_detected - doorbell heard
        timeout          - no doorbell detected within timeout
    """

    def __init__(self,
                 doorbell_topic='/audio_detection/doorbell',
                 timeout=60.0):
        smach.State.__init__(
            self,
            outcomes=['doorbell_detected', 'timeout']
        )
        self.doorbell_topic = doorbell_topic
        self.timeout = timeout
        self.doorbell_heard = False

    def _doorbell_callback(self, msg):
        self.doorbell_heard = True

    def execute(self, userdata):
        rospy.loginfo('[WaitForDoorbell] Waiting for doorbell...')
        
        from std_msgs.msg import Bool
        self.doorbell_heard = False
        sub = rospy.Subscriber(self.doorbell_topic, Bool, self._doorbell_callback)
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.doorbell_heard:
                sub.unregister()
                rospy.loginfo('[WaitForDoorbell] Doorbell detected!')
                return 'doorbell_detected'
            
            if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                sub.unregister()
                rospy.logwarn('[WaitForDoorbell] Timeout, no doorbell')
                return 'timeout'
            
            rate.sleep()
        
        sub.unregister()
        return 'timeout'

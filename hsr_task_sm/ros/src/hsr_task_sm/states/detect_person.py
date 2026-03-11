#!/usr/bin/env python3
"""
DetectPerson smach state - Person detection and recognition.

Detects people in the robot's field of view.
"""

import rospy
import smach

from geometry_msgs.msg import PoseStamped


class DetectPerson(smach.State):
    """
    Detect people in front of the robot.
    
    Outcomes:
        succeeded              - person(s) detected
        no_person_found        - no person detected
        failed_after_retrying  - max retries exhausted
    Output userdata:
        detected_persons (list) - list of detected person info dicts
    """

    def __init__(self,
                 detection_topic='/person_detection/detections',
                 timeout=5.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'no_person_found', 'failed_after_retrying'],
            output_keys=['detected_persons']
        )
        self.detection_topic = detection_topic
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

    def execute(self, userdata):
        rospy.loginfo('[DetectPerson] Looking for people...')
        
        # Wait for detection message
        try:
            from mas_perception_msgs.msg import PersonList
            msg = rospy.wait_for_message(
                self.detection_topic,
                PersonList,
                timeout=self.timeout
            )
            
            if msg.persons and len(msg.persons) > 0:
                rospy.loginfo('[DetectPerson] Found %d person(s)', len(msg.persons))
                userdata.detected_persons = [
                    {
                        'id': p.id,
                        'name': p.name if hasattr(p, 'name') else 'unknown',
                        'pose': p.pose
                    }
                    for p in msg.persons
                ]
                self.retry_count = 0
                return 'succeeded'
                
        except rospy.ROSException:
            rospy.logwarn('[DetectPerson] Detection timed out')
        except Exception as e:
            rospy.logerr('[DetectPerson] Error: %s', str(e))
        
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[DetectPerson] Retry %d/%d', self.retry_count, self.retries)
        return 'no_person_found'


class DetectWavingPerson(smach.State):
    """
    Detect a person who is waving or calling for attention.
    
    Outcomes:
        succeeded              - waving person detected
        no_waving_person       - no waving person found
        failed_after_retrying  - max retries exhausted
    Output userdata:
        waving_person (dict) - info about the waving person
    """

    def __init__(self,
                 gesture_topic='/gesture_detection/waving',
                 timeout=10.0,
                 retries=3):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'no_waving_person', 'failed_after_retrying'],
            output_keys=['waving_person']
        )
        self.gesture_topic = gesture_topic
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

    def execute(self, userdata):
        rospy.loginfo('[DetectWavingPerson] Looking for waving/calling customer...')
        
        # Placeholder - integrate with actual gesture detection
        try:
            from geometry_msgs.msg import PoseStamped
            msg = rospy.wait_for_message(
                self.gesture_topic,
                PoseStamped,
                timeout=self.timeout
            )
            
            rospy.loginfo('[DetectWavingPerson] Found waving person!')
            userdata.waving_person = {
                'pose': msg,
                'gesture': 'waving'
            }
            self.retry_count = 0
            return 'succeeded'
            
        except rospy.ROSException:
            rospy.logwarn('[DetectWavingPerson] No waving person detected')
        
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        return 'no_waving_person'

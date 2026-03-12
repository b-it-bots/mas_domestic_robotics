#!/usr/bin/env python3
"""
Speak smach state - Text-to-Speech output.

Publishes to /say topic for TTS.
"""

import rospy
import smach

from std_msgs.msg import String


class Speak(smach.State):
    """
    Speak a given text using TTS via /say topic.
    
    Outcomes:
        succeeded              - speech completed
        failed                 - TTS failed; will retry up to `retries` times
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self, text=None, text_key=None, text_prefix=None, text_suffix=None, topic='/say', retries=2):
        """
        Args:
            text: Static text to speak (optional)
            text_key: Userdata key containing text to speak (optional)
            text_prefix: Static prefix prepended before text_key value (optional)
            text_suffix: Static suffix appended after text_key value (optional)
            topic: Topic to publish text to (default: /say)
            retries: Number of retries on failure (default: 2)
        """
        input_keys = [text_key] if text_key else []
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=input_keys
        )
        self.text = text
        self.text_key = text_key
        self.text_prefix = text_prefix or ''
        self.text_suffix = text_suffix or ''
        self.topic = topic
        self.retries = retries
        self.retry_count = 0
        self.pub = None

    def execute(self, userdata):
        if self.pub is None:
            self.pub = rospy.Publisher(self.topic, String, queue_size=1)
            rospy.sleep(0.3)  # Wait for publisher to connect
        
        if self.text_key:
            text_to_speak = self.text_prefix + str(getattr(userdata, self.text_key)) + self.text_suffix
        else:
            text_to_speak = self.text
            
        if not text_to_speak:
            rospy.logwarn('[Speak] No text to speak')
            return self._retry()
            
        rospy.loginfo('[Speak] Saying: "%s"', text_to_speak)
        
        try:
            self.pub.publish(String(data=text_to_speak))
            rospy.sleep(0.5)  # Brief pause after speaking
            self.retry_count = 0
            return 'succeeded'
        except Exception as e:
            rospy.logerr('[Speak] TTS failed: %s', str(e))
            return self._retry()

    def _retry(self):
        """Handle retry logic."""
        if self.retry_count >= self.retries:
            self.retry_count = 0
            rospy.logerr('[Speak] Max retries (%d) reached', self.retries)
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[Speak] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'

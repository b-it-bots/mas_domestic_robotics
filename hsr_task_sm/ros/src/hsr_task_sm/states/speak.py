#!/usr/bin/env python3
"""
Speak smach state - Text-to-Speech output.

Uses sound_play or a custom TTS service to speak text.
"""

import rospy
import smach

from sound_play.libsoundplay import SoundClient


class Speak(smach.State):
    """
    Speak a given text using TTS.
    
    Outcomes:
        succeeded              - speech completed
        failed                 - TTS failed; will retry up to `retries` times
        failed_after_retrying  - max retries exhausted
    """

    def __init__(self, text=None, text_key=None, blocking=True, retries=2):
        """
        Args:
            text: Static text to speak (optional)
            text_key: Userdata key containing text to speak (optional)
            blocking: Wait for speech to complete (default: True)
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
        self.blocking = blocking
        self.retries = retries
        self.retry_count = 0
        
        self.sound_client = SoundClient(blocking=blocking)
        rospy.sleep(0.5)  # Wait for sound_play to initialize

    def execute(self, userdata):
        if self.text_key:
            text_to_speak = getattr(userdata, self.text_key)
        else:
            text_to_speak = self.text
            
        if not text_to_speak:
            rospy.logwarn('[Speak] No text to speak')
            return self._retry()
            
        rospy.loginfo('[Speak] Saying: "%s"', text_to_speak)
        
        try:
            self.sound_client.say(text_to_speak)
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

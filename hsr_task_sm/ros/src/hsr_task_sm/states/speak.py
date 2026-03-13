#!/usr/bin/env python3
"""
Speak smach state - Text-to-Speech output.

Publishes to /say topic for TTS.
"""

import rospy
import smach

from std_msgs.msg import String
import re


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


    # ── TTS timing ─────────────────────────────────────────────────────────────
    #
    # HSR uses a TTS engine with a measured speaking rate of ~130 wpm.
    # We add punctuation pauses on top so "Hello. What is your name?" waits
    # longer than a same-length unpunctuated string.
    #
    # To recalibrate for your robot:
    #   1. Call _say("one two three four five")   (5 words, no punctuation)
    #   2. Time how long the robot actually speaks
    #   3. Set _TTS_SECS_PER_WORD = measured_seconds / 5
    #
    _TTS_SECS_PER_WORD = 0.43   # ~130 wpm
    _TTS_PAUSE_PERIOD  = 0.45   # extra pause per  .  !  ?
    _TTS_PAUSE_COMMA   = 0.20   # extra pause per  ,  ;  :
    _TTS_TAIL_BUFFER   = 0.35   # silence after last word before mic opens
    _TTS_MIN_DURATION  = 1.0    # floor for very short utterances
 
    def _tts_duration(self, text: str) -> float:
        """
        Estimate how many seconds the HSR will take to speak *text*.
 
        duration = (words x secs_per_word)
                 + (sentence-end punctuation x pause_period)
                 + (mid-sentence punctuation x pause_comma)
                 + tail_buffer
        """
        words         = len(text.split())
        sentence_ends = len(re.findall(r'[.!?]', text))
        mid_pauses    = len(re.findall(r'[,;:]',  text))
        duration = (
            words         * self._TTS_SECS_PER_WORD
            + sentence_ends * self._TTS_PAUSE_PERIOD
            + mid_pauses    * self._TTS_PAUSE_COMMA
            + self._TTS_TAIL_BUFFER
        )
        if duration > 10:
            return 10
        else:
            return max(self._TTS_MIN_DURATION, duration)

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
            sleep=self._tts_duration(text_to_speak)
            rospy.logwarn(f'[Speak] Sleep count {sleep}')
            rospy.sleep(sleep)  # Brief pause after speaking
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

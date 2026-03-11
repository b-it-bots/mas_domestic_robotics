#!/usr/bin/env python3
"""
ListenForCommand smach state - Speech recognition.

Listens for spoken commands using speech recognition.
"""

import rospy
import smach

from std_msgs.msg import String


class ListenForCommand(smach.State):
    """
    Listen for a spoken command using ASR.
    
    Outcomes:
        succeeded            - command understood
        failed               - ASR failed
        failed_after_retrying - max retries exhausted
    Output userdata:
        recognized_command (str) - the recognized text
    """

    def __init__(self,
                 asr_topic='/speech_recognition/final_result',
                 timeout=10.0,
                 retries=3):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            output_keys=['recognized_command']
        )
        self.asr_topic = asr_topic
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0
        self.recognized_text = None

    def _asr_callback(self, msg):
        self.recognized_text = msg.data

    def execute(self, userdata):
        self.recognized_text = None
        
        rospy.loginfo('[ListenForCommand] Listening for command...')
        sub = rospy.Subscriber(self.asr_topic, String, self._asr_callback)
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            if self.recognized_text:
                break
            if (rospy.Time.now() - start_time).to_sec() > self.timeout:
                break
            rate.sleep()
        
        sub.unregister()
        
        if self.recognized_text and len(self.recognized_text.strip()) > 0:
            rospy.loginfo('[ListenForCommand] Recognized: "%s"', self.recognized_text)
            userdata.recognized_command = self.recognized_text
            self.retry_count = 0
            return 'succeeded'
        
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            rospy.logerr('[ListenForCommand] Max retries reached')
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[ListenForCommand] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'

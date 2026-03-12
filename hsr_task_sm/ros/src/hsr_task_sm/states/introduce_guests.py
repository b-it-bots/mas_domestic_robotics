#!/usr/bin/env python3
"""
IntroduceGuests state - Introduce guests to each other.

Reads saved guest JSON files and introduces each guest to the other.
"""

import os
import json
import rospy
import smach

from std_msgs.msg import String


class IntroduceGuests(smach.State):
    """
    Introduce guests to each other by reading their saved info.
    
    Reads from guest JSON files and speaks introductions.
    
    Params:
        json_dir: directory containing guest JSON files
        retries: max retries before failing
    
    Outcomes:
        succeeded               - introductions completed
        failed                  - temporary failure (retry)
        failed_after_retrying   - max retries exhausted
    """

    def __init__(self,
                 json_dir='/tmp/hri_guests',
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying']
        )
        self.json_dir = json_dir
        self.retries = retries
        self.retry_count = 0
        
        # Publisher for TTS
        self.say_pub = rospy.Publisher('/say', String, queue_size=10)

    def _say(self, text):
        """Publish text to /say topic."""
        rospy.loginfo('[IntroduceGuests] Saying: %s', text)
        self.say_pub.publish(String(data=text))
        # Calculate delay based on word count
        num_words = len(text.split())
        delay = max(1.0, num_words * 0.5)  # longer delay for introductions
        rospy.sleep(delay)

    def execute(self, userdata):
        rospy.loginfo('[IntroduceGuests] Starting introductions...')
        
        # Load guest info from JSON files
        json1_path = os.path.join(self.json_dir, 'person1.json')
        json2_path = os.path.join(self.json_dir, 'person2.json')
        
        try:
            with open(json1_path) as f:
                data1 = json.load(f)
                person1 = data1.get('guest1', {})
        except (FileNotFoundError, json.JSONDecodeError) as e:
            rospy.logwarn('[IntroduceGuests] Could not load person1.json: %s', e)
            person1 = None
            
        try:
            with open(json2_path) as f:
                data2 = json.load(f)
                person2 = data2.get('guest1', {})
        except (FileNotFoundError, json.JSONDecodeError) as e:
            rospy.logwarn('[IntroduceGuests] Could not load person2.json: %s', e)
            person2 = None
        
        if not person1 or not person2:
            rospy.logerr('[IntroduceGuests] Missing guest information')
            return self._retry()
        
        name1 = person1.get('name', 'Guest 1')
        drink1 = person1.get('drink', 'unknown drink')
        name2 = person2.get('name', 'Guest 2')
        drink2 = person2.get('drink', 'unknown drink')
        
        rospy.loginfo('[IntroduceGuests] Person1: %s (%s), Person2: %s (%s)',
                      name1, drink1, name2, drink2)
        
        # Introduce person 2 to person 1
        intro1 = f"{name1}, I would like to introduce you to {name2}, whose favorite drink is {drink2}."
        self._say(intro1)
        
        rospy.sleep(1.0)  # pause between introductions
        
        # Introduce person 1 to person 2
        intro2 = f"{name2}, I would like to introduce you to {name1}, whose favorite drink is {drink1}."
        self._say(intro2)
        
        rospy.loginfo('[IntroduceGuests] Introductions completed')
        self.retry_count = 0
        return 'succeeded'

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[IntroduceGuests] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'

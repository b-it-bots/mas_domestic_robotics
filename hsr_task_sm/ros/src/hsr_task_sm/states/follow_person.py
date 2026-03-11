#!/usr/bin/env python3
"""
FollowPerson smach state - Follow a person.

Follows a detected person until they stop.
"""

import rospy
import smach
import actionlib

from geometry_msgs.msg import Twist, PoseStamped


class FollowPerson(smach.State):
    """
    Follow a person until they signal to stop.
    
    Outcomes:
        succeeded           - arrived at destination (person stopped)
        lost_person         - lost track of the person
        failed              - following failed
    """

    def __init__(self,
                 cmd_vel_topic='/cmd_vel',
                 person_topic='/person_tracker/target',
                 stop_distance=1.0,
                 timeout=120.0):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'lost_person', 'failed'],
            input_keys=['person_to_follow']  # optional initial person
        )
        self.stop_distance = stop_distance
        self.timeout = timeout
        self.person_pose = None
        
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.person_sub = None
        self.person_topic = person_topic

    def _person_callback(self, msg):
        self.person_pose = msg

    def execute(self, userdata):
        rospy.loginfo('[FollowPerson] Starting to follow person...')
        
        self.person_sub = rospy.Subscriber(
            self.person_topic, 
            PoseStamped, 
            self._person_callback
        )
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        lost_count = 0
        prev_pose = None
        stationary_count = 0
        
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.timeout:
                rospy.logwarn('[FollowPerson] Timeout reached')
                break
            
            if self.person_pose is None:
                lost_count += 1
                if lost_count > 30:  # Lost for 3 seconds
                    self.person_sub.unregister()
                    return 'lost_person'
                rate.sleep()
                continue
            
            lost_count = 0
            
            # Check if person has stopped
            if prev_pose:
                dx = self.person_pose.pose.position.x - prev_pose.pose.position.x
                dy = self.person_pose.pose.position.y - prev_pose.pose.position.y
                if abs(dx) < 0.05 and abs(dy) < 0.05:
                    stationary_count += 1
                    if stationary_count > 20:  # Stationary for 2 seconds
                        rospy.loginfo('[FollowPerson] Person stopped, arrived!')
                        self._stop()
                        self.person_sub.unregister()
                        return 'succeeded'
                else:
                    stationary_count = 0
            
            prev_pose = self.person_pose
            
            # Simple proportional control to follow
            cmd = Twist()
            x = self.person_pose.pose.position.x
            y = self.person_pose.pose.position.y
            
            distance = (x**2 + y**2)**0.5
            
            if distance > self.stop_distance:
                cmd.linear.x = min(0.3, 0.5 * (distance - self.stop_distance))
                cmd.angular.z = 0.8 * y / max(distance, 0.1)
            
            self.cmd_pub.publish(cmd)
            rate.sleep()
        
        self._stop()
        if self.person_sub:
            self.person_sub.unregister()
        return 'failed'

    def _stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

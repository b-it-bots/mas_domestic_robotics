#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib

from geometry_msgs.msg import Twist
from mdr_move_forward_action.msg import MoveForwardFeedback, MoveForwardResult


class SetupMoveForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['move_forward_goal'],
                             output_keys=['move_forward_feedback', 'move_forward_result'])

    def execute(self, userdata):
        duration = userdata.move_forward_goal.movement_duration
        speed = userdata.move_forward_goal.speed

        feedback = MoveForwardFeedback()
        feedback.current_state = 'MOVE_FORWARD'
        feedback.message = '[move_forward] moving the base forward for ' + str(duration) + 's at ' + str(speed) + 'm/s'
        userdata.move_forward_feedback = feedback

        return 'succeeded'


class MoveForward(smach.State):
    def __init__(self, timeout=120.0, velocity_topic='/base/twist_mux/command_navigation'):
        smach.State.__init__(self, input_keys=['move_forward_goal'], outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.velocity_pub = rospy.Publisher(velocity_topic, Twist)

    def execute(self, userdata):
        duration = rospy.Duration.from_sec(userdata.move_forward_goal.movement_duration)
        speed = userdata.move_forward_goal.speed

        rate = rospy.Rate(5)
        twist = Twist()
        twist.linear.x = speed

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < duration:
            self.velocity_pub.publish(twist)
            rate.sleep()

        # we publish a zero twist at the end so that the robot stops moving
        zero_twist = Twist()
        self.velocity_pub.publish(zero_twist)

        return 'succeeded'


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['move_forward_goal'],
                             output_keys=['move_forward_feedback', 'move_forward_result'])
        self.result = result

    def execute(self, userdata):
        result = MoveForwardResult()
        result.success = self.result
        userdata.move_forward_result = result
        return 'succeeded'

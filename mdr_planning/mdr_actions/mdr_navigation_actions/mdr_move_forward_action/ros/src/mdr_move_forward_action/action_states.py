#!/usr/bin/python
import rospy

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from geometry_msgs.msg import Twist
from mdr_move_forward_action.msg import MoveForwardFeedback, MoveForwardResult

class MoveForwardSM(ActionSMBase):
    def __init__(self, timeout=120.0, velocity_topic='/cmd_vel', max_recovery_attempts=1):
        super(MoveForwardSM, self).__init__('MoveForward', [], max_recovery_attempts)
        self.timeout = timeout
        self.velocity_pub = rospy.Publisher(velocity_topic, Twist)

    def running(self):
        duration = rospy.Duration.from_sec(self.goal.movement_duration)
        speed = self.goal.speed

        rate = rospy.Rate(5)
        twist = Twist()
        twist.linear.x = speed

        rospy.loginfo('[move_forward] moving the base forward for %s s at %s m/s', str(duration), str(speed))
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) < duration:
            self.velocity_pub.publish(twist)
            rate.sleep()

        # we publish a zero twist at the end so that the robot stops moving
        zero_twist = Twist()
        self.velocity_pub.publish(zero_twist)
        self.result = self.set_result(True)
        return FTSMTransitions.DONE

    def set_result(self, success):
        result = MoveForwardResult()
        result.success = success
        return result

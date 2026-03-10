#!/usr/bin/env python3
"""
GoToGoal smach state.

Sends an (x, y, theta) pose goal to the standard ROS move_base action server.
"""

import math

import rospy
import smach
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, Quaternion


def _yaw_to_quaternion(yaw_rad):
    """Convert a yaw angle (rad) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


class GoToGoal(smach.State):
    """
    Navigate to an explicit (x, y, theta) pose via the standard move_base server.

    Parameters
    ----------
    x, y : float
        Target position in the map frame (metres).
    theta : float
        Target heading in radians.
    frame_id : str
        Reference frame for the goal (default: "map").
    server_name : str
        Action server name (default: "move_base").
    timeout : float
        Seconds to wait for the action to complete (default: 120.0).
    retries : int
        Number of times to retry on failure before giving up (default: 2).

    Outcomes
    --------
    succeeded             – goal reached
    failed                – navigation failed; will retry up to `retries` times
    failed_after_retrying – max retries exhausted
    """

    def __init__(self,
                 x,
                 y,
                 theta,
                 frame_id='map',
                 server_name='move_base',
                 timeout=120.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying']
        )
        self.x = x
        self.y = y
        self.theta = theta
        self.frame_id = frame_id
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

        rospy.loginfo('[GoToGoal] Connecting to %s ...', server_name)
        self.client = actionlib.SimpleActionClient(server_name, MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('[GoToGoal] Connected.')

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.target_pose = PoseStamped()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.x
        goal.target_pose.pose.position.y = self.y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = _yaw_to_quaternion(self.theta)

        rospy.loginfo('[GoToGoal] Navigating to (%.2f, %.2f, %.2f rad) in "%s" ...',
                      self.x, self.y, self.theta, self.frame_id)
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(self.timeout))
        if not finished:
            self.client.cancel_goal()
            rospy.logerr('[GoToGoal] Timed out after %.0fs', self.timeout)
            return self._retry()

        state = self.client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo('[GoToGoal] Reached (%.2f, %.2f)', self.x, self.y)
            self.retry_count = 0
            return 'succeeded'

        rospy.logwarn('[GoToGoal] Failed with action state %d', state)
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            rospy.logerr('[GoToGoal] Max retries (%d) reached', self.retries)
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[GoToGoal] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'

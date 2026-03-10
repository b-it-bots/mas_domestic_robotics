#!/usr/bin/env python3
"""
NavigateTo smach state.

Sends a named destination to /move_base_server via actionlib.
No ROSPlan KB involved.
"""

import rospy
import smach
import actionlib

from mdr_move_base_action.msg import MoveBaseAction, MoveBaseGoal


class NavigateTo(smach.State):
    """
    Outcomes:
        succeeded              -- destination reached
        failed                 -- navigation failed; will retry up to `retries` times
        failed_after_retrying  -- max retries exhausted
    """

    def __init__(self,
                 destination,
                 server_name='move_base_server',
                 timeout=120.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying']
        )
        self.destination = destination
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

        rospy.loginfo('[NavigateTo] Connecting to %s ...', server_name)
        self.client = actionlib.SimpleActionClient(server_name, MoveBaseAction)
        self.client.wait_for_server()
        rospy.loginfo('[NavigateTo] Connected.')

    def execute(self, userdata):
        goal = MoveBaseGoal()
        goal.goal_type = MoveBaseGoal.NAMED_TARGET
        goal.destination_location = self.destination

        rospy.loginfo('[NavigateTo] Navigating to "%s" ...', self.destination)
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(self.timeout))
        if not finished:
            self.client.cancel_goal()
            rospy.logerr('[NavigateTo] Timed out after %.0fs', self.timeout)
            return self._retry()

        result = self.client.get_result()
        if result and result.success:
            rospy.loginfo('[NavigateTo] Reached "%s"', self.destination)
            self.retry_count = 0
            return 'succeeded'

        rospy.logwarn('[NavigateTo] Failed to reach "%s"', self.destination)
        return self._retry()

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            rospy.logerr('[NavigateTo] Max retries (%d) reached', self.retries)
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[NavigateTo] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'

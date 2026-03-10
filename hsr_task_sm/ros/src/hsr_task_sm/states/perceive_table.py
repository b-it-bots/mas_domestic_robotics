#!/usr/bin/env python3
"""
PerceiveTable smach state.

Calls /mdr_actions/perceive_plane_server directly via actionlib.
Stores the full PlaneList result in userdata so downstream states
can access object poses without any KB or datacentre lookup.
"""

import rospy
import smach
import actionlib

from mdr_perceive_plane_action.msg import PerceivePlaneAction, PerceivePlaneGoal


class PerceiveTable(smach.State):
    """
    Outcomes:
        succeeded             -- at least one object detected on a plane
        failed                -- detection failed; will retry up to `retries` times
        failed_after_retrying -- max retries exhausted
    Output userdata:
        perceived_planes (mas_perception_msgs/PlaneList)
    """

    def __init__(self,
                 plane_prefix='table',
                 server_name='/mdr_actions/perceive_plane_server',
                 timeout=30.0,
                 retries=2):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            output_keys=['perceived_planes']
        )
        self.plane_prefix = plane_prefix
        self.timeout = timeout
        self.retries = retries
        self.retry_count = 0

        rospy.loginfo('[PerceiveTable] Connecting to %s ...', server_name)
        self.client = actionlib.SimpleActionClient(server_name, PerceivePlaneAction)
        self.client.wait_for_server()
        rospy.loginfo('[PerceiveTable] Connected.')

    def execute(self, userdata):
        goal = PerceivePlaneGoal()
        goal.plane_config = ''
        goal.plane_frame_prefix = self.plane_prefix

        rospy.loginfo('[PerceiveTable] Sending goal (plane_frame_prefix="%s")', self.plane_prefix)
        self.client.send_goal(goal)
        finished = self.client.wait_for_result(rospy.Duration(self.timeout))

        if not finished:
            rospy.logerr('[PerceiveTable] Timed out after %.1fs', self.timeout)
            return self._retry()

        result = self.client.get_result()
        if not result or not result.success:
            rospy.logerr('[PerceiveTable] Server reported failure')
            return self._retry()

        planes = result.recognized_planes.planes
        planes_with_objects = [p for p in planes if len(p.object_list.objects) > 0]

        if not planes_with_objects:
            rospy.logwarn('[PerceiveTable] No objects found on any plane')
            return self._retry()

        rospy.loginfo('[PerceiveTable] Found %d plane(s) with objects:', len(planes_with_objects))
        for p in planes_with_objects:
            obj_strs = ['%s (%.3f)' % (o.name, o.probability) for o in p.object_list.objects]
            rospy.loginfo('[PerceiveTable]   plane="%s" z=%.3f  objects: %s',
                          p.name, p.plane_point.z, ', '.join(obj_strs))

        self.retry_count = 0
        userdata.perceived_planes = result.recognized_planes
        return 'succeeded'

    def _retry(self):
        if self.retry_count >= self.retries:
            self.retry_count = 0
            rospy.logerr('[PerceiveTable] Max retries (%d) reached, giving up', self.retries)
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[PerceiveTable] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'

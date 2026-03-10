#!/usr/bin/env python3
"""
ClearCostmap smach state.

Calls /move_base/clear_costmaps (std_srvs/Empty) to remove accumulated
dynamic obstacle inflation from the costmap before navigation or perception.
Unlike the 'clear_obs' dynparam alias, this only clears the dynamic layer
and does not change the obstacle inflation radius, so static surface margins
are preserved.
"""

import rospy
import smach
from std_srvs.srv import Empty, EmptyRequest


class ClearCostmap(smach.State):
    """
    Outcomes:
        succeeded  -- costmap cleared (or service unavailable, warn and continue)
        failed     -- unexpected exception
    """

    def __init__(self, service_name='/move_base/clear_costmaps', timeout=5.0):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.service_name = service_name
        self.timeout = timeout

    def execute(self, userdata):
        rospy.loginfo('[ClearCostmap] Waiting for %s ...', self.service_name)
        try:
            rospy.wait_for_service(self.service_name, timeout=self.timeout)
            clear = rospy.ServiceProxy(self.service_name, Empty)
            clear(EmptyRequest())
            rospy.loginfo('[ClearCostmap] Costmap cleared successfully.')
            return 'succeeded'
        except rospy.ROSException:
            rospy.logwarn('[ClearCostmap] %s not available within %.1fs; continuing without clear.',
                          self.service_name, self.timeout)
            return 'succeeded'
        except Exception as exc:
            rospy.logerr('[ClearCostmap] Unexpected error: %s', exc)
            return 'failed'

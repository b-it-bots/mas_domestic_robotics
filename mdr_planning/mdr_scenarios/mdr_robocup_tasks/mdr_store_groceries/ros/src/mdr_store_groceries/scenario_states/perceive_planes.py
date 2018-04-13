import time
import rospy

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs

from mdr_store_groceries.scenario_states.scenario_state_base import ScenarioStateBase

class PerceivePlanes(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'perceive_plane',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', 'mdr_store_groceries')
        self.state_name = kwargs.get('state_name', 'perceive_planes')
        self.timeout = kwargs.get('timeout', 120.)
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.plane_prefix = kwargs.get('plane_prefix', 0)

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        dispatch_msg = self.get_dispatch_msg(self.plane_prefix)
        rospy.loginfo('Perceiving plane %s' % self.plane_prefix)
        self.action_dispatch_pub.publish(dispatch_msg)

        self.executing = True
        self.succeeded = False
        start_time = time.time()
        duration = 0.
        while self.executing and duration < self.timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.succeeded:
            rospy.loginfo('%s perceived successfully' % self.plane_prefix)
            return 'succeeded'

        rospy.loginfo('Could not perceive %s' % self.plane_prefix)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to perceive %s' % self.plane_prefix)
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to perceive %s' % self.plane_prefix)
        self.retry_count += 1
        return 'failed'

    def get_dispatch_msg(self, plane_name):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = self.action_name

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'bot'
        arg_msg.value = self.robot_name
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'plane'
        arg_msg.value = plane_name
        dispatch_msg.parameters.append(arg_msg)

        return dispatch_msg

import time
import numpy as np
import rospy
from std_msgs.msg import String

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import rosplan_knowledge_msgs.srv as rosplan_srvs
import diagnostic_msgs.msg as diag_msgs

from mcr_perception_msgs.msg import Object
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class Throw(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'throw',
                                   save_sm_state=save_sm_state,
                                   input_keys=['grasped_object'],
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.sm_id = kwargs.get('sm_id', 'mdr_demo_throw_table_objects')
        self.state_name = kwargs.get('state_name', 'throw')
        self.throwing_target_name = kwargs.get('state_name', 'throwing_target_name')
        self.timeout = kwargs.get('timeout', 120.)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

    def execute(self, userdata):
        if self.save_sm_state:
            self.save_current_state()

        grasped_object = userdata.grasped_object
        dispatch_msg = self.get_dispatch_msg(grasped_object)
        rospy.loginfo('Throwing %s in the trash can' % grasped_object)
        self.say('Throwing ' + grasped_object + ' in the trash can')
        self.action_dispatch_pub.publish(dispatch_msg)

        self.executing = True
        self.succeeded = False
        start_time = time.time()
        duration = 0.
        while self.executing and duration < self.timeout:
            rospy.sleep(0.1)
            duration = time.time() - start_time

        if self.succeeded:
            rospy.loginfo('Object discarded successfully')
            self.say('Successfully discarded ' + grasped_object)
            if self.surface_empty(surface_name='table'):
                return 'finished'
            return 'pick_new_object'

        rospy.loginfo('Could not discard object %s' % grasped_object)
        self.say('Could not discard ' + grasped_object)
        if self.retry_count == self.number_of_retries:
            rospy.loginfo('Failed to discard object %s' % grasped_object)
            self.say('Aborting operation')
            return 'failed_after_retrying'
        rospy.loginfo('Retrying to discard object %s' % grasped_object)
        self.retry_count += 1
        return 'failed'

    def get_dispatch_msg(self, obj_name):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = self.action_name

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'bot'
        arg_msg.value = self.robot_name
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'obj'
        arg_msg.value = obj_name
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'throwing_target'
        arg_msg.value = self.throwing_target_name
        dispatch_msg.parameters.append(arg_msg)

        return dispatch_msg

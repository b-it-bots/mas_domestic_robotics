import time
import rospy
import actionlib

import rosplan_dispatch_msgs.msg as plan_dispatch_msgs
import diagnostic_msgs.msg as diag_msgs

from mdr_move_base_action.msg import MoveBaseAction
from mas_execution_manager.scenario_state_base import ScenarioStateBase

class MoveBase(ScenarioStateBase):
    def __init__(self, save_sm_state=False, **kwargs):
        ScenarioStateBase.__init__(self, 'move_base',
                                   save_sm_state=save_sm_state,
                                   outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                                   input_keys=['destination_locations'])
        self.sm_id = kwargs.get('sm_id', '')
        self.state_name = kwargs.get('state_name', 'move_base')
        self.move_base_server = kwargs.get('move_base_server', 'move_base_server')
        self.destination_locations = list(kwargs.get('destination_locations', list()))
        self.timeout = kwargs.get('timeout', 120.)

        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0

        self.move_base_client = actionlib.SimpleActionClient(self.move_base_server,
                                                             MoveBaseAction)
        self.move_base_client.wait_for_server()

    def execute(self, userdata):
        original_location = self.kb_interface.get_robot_location(self.robot_name)
        if len(self.destination_locations) == 0:
            self.destination_locations = userdata.destination_locations
            rospy.loginfo("Using userdata's destination_locations {0}".format(self.destination_locations))

        for destination_location in self.destination_locations:
            dispatch_msg = self.get_dispatch_msg(original_location,
                                                 destination_location)

            rospy.loginfo('Sending the base to %s' % destination_location)
            self.say('Going to ' + destination_location)
            self.action_dispatch_pub.publish(dispatch_msg)

            self.executing = True
            self.succeeded = False
            start_time = time.time()
            duration = 0.
            while self.executing and duration < self.timeout:
                rospy.sleep(0.1)
                duration = time.time() - start_time

            if self.succeeded:
                rospy.loginfo('Successfully reached %s' % destination_location)
                original_location = destination_location
            else:
                rospy.logerr('Could not reach %s' % destination_location)
                self.say('Could not reach ' + destination_location)
                if self.retry_count == self.number_of_retries:
                    self.say('Aborting operation')
                    return 'failed_after_retrying'
                self.retry_count += 1
                return 'failed'
        return 'succeeded'

    def get_dispatch_msg(self, original_location, destination_location):
        dispatch_msg = plan_dispatch_msgs.ActionDispatch()
        dispatch_msg.name = self.action_name

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'bot'
        arg_msg.value = self.robot_name
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'from'
        arg_msg.value = original_location
        dispatch_msg.parameters.append(arg_msg)

        arg_msg = diag_msgs.KeyValue()
        arg_msg.key = 'to'
        arg_msg.value = destination_location
        dispatch_msg.parameters.append(arg_msg)

        return dispatch_msg

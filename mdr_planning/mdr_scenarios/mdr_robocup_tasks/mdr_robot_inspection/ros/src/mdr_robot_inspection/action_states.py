#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib

from std_msgs.msg import String

from mdr_enter_door_action.msg import EnterDoorAction, EnterDoorGoal
from mdr_move_base_safe.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mdr_move_tray_action.msg import MoveTrayAction, MoveTrayGoal


class Enter(smach.State):
    def __init__(self, timeout=120., enter_action_server='/mdr_actions/enter_action_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['_feedback'])
        self.enter_action_client = actionlib.SimpleActionClient(enter_action_server, EnterDoorAction)
        self.enter_action_client.wait_for_server()

    def execute(self, userdata):
        goal = EnterDoorGoal()

        self.enter_action_client.send_goal(goal)
        result = self.enter_action_client.wait_for_result(rospy.Duration.from_sec(self.timeout))

        if result and result.success:
            return 'succeeded'
        else:
            return 'failed'


class MoveBase(smach.State):
    def __init__(self, destination_locations, timeout=120.0, action_server='/mdr_actions/move_base_safe_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.destination_locations = list(destination_locations)
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, MoveBaseSafeAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = MoveBaseSafeGoal()
        for i, destination_location in enumerate(self.destination_locations):
            if i == 0:
                goal.source_location = 'anywhere'
            else:
                goal.source_location = self.destination_locations[i-1]
            goal.destination_location = destination_location
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))

            res = self.client.get_result()
            if res and res.success:
                rospy.loginfo('Successfully reached ' % destination_location)
            else:
                rospy.logerror('Could not reach ' % destination_location)
                return 'failed'
        return 'succeeded'


class MoveTray(smach.State):
    def __init__(self, direction, timeout=120.0, action_server='/mdr_actions/move_tray_server'):
        smach.State.__init__(self, outcomes=['up', 'down', 'failed'])
        self.direction = direction
        self.timeout = timeout

        self.move_tray_client = actionlib.SimpleActionClient(action_server, MoveTrayAction)
        self.move_tray_client.wait_for_server()

    def execute(self, userdata):
        goal = MoveTrayGoal()
        goal.direction = self.direction

        self.move_tray_client.send_goal(goal)
        self.move_tray_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return self.direction
        else:
            return 'failed'


class WaitForQR(smach.State):
    def __init__(self, timeout=120.0, qr_topic='/mcr_perception/qr_reader/output'):
        smach.State.__init__(self, outcomes=['succeeded', 'waiting', 'failed'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.state_check_rate = rospy.Rate(5)
        self.qr_code_sub = rospy.Subscriber(qr_topic, String, self.register_qr_code)
        self.qr_message = None
        self.start_time = rospy.Time.now()

    def execute(self, userdata):
        if (rospy.Time.now() - self.start_time) < timeout:
            if self.qr_message and self.qr_message.lower().find('continue') != -1:
                rospy.loginfo('QR message: %s' % self.qr_message)
                return 'succeeded'
            else:
                self.state_check_rate.sleep()
                rospy.loginfo('Waiting for QR code')
                return 'waiting'
        else:
            rospy.loginfo('wait_for_qr timed out')
            return 'failed'

    def register_qr_code(self, msg):
        self.qr_message = msg.data

#!/usr/bin/python

import rospy
import smach
import smach_ros
from actionlib import SimpleActionClient

from std_msgs.msg import Bool
from mdr_enter_door_action.msg import EnterDoorFeedback, EnterDoorResult
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal


class SetupEnterDoor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['enter_door_feedback'])

    def execute(self, userdata):
        feedback = EnterDoorFeedback()
        feedback.current_state = 'setup_enter_door'
        feedback.message = '[enter_door] entering door'
        userdata.enter_door_feedback = feedback

        return 'succeeded'


class WaitForDoor(smach.State):
    def __init__(self, sleep_duration=1.,
                 door_status_topic='/mcr_perception/door_status/door_status'):
        smach.State.__init__(self, outcomes=['succeeded', 'waiting'],
                             output_keys=['enter_door_feedback'])
        self.sleep_duration = sleep_duration
        self.door_open = False
        self.door_status_sub = rospy.Subscriber(door_status_topic, Bool,
                                                self.update_door_status)

    def execute(self, userdata):
        feedback = EnterDoorFeedback()
        feedback.current_state = 'wait_for_door'
        feedback.message = '[enter_door] waiting for door to open'
        userdata.enter_door_feedback = feedback

        rospy.sleep(self.sleep_duration)

        if self.door_open:
            return 'succeeded'
        else:
            return 'waiting'

    def update_door_status(self, msg):
        self.door_open = msg.data


class EnterDoor(smach.State):
    def __init__(self, timeout=120.0,
                 move_forward_server='/mdr_actions/move_forward_server',
                 movement_duration=15., speed=0.1):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.movement_duration = movement_duration
        self.speed = speed

        self.move_forward_client = SimpleActionClient(move_forward_server,
                                                      MoveForwardAction)
        self.move_forward_client.wait_for_server()

        self.entered = False

    def execute(self, userdata):
        goal = MoveForwardGoal()
        goal.movement_duration = self.movement_duration
        goal.speed = self.speed

        self.move_forward_client.send_goal(goal)
        timeout_duration = rospy.Duration.from_sec(self.timeout)
        self.move_forward_client.wait_for_result(timeout_duration)

        result = self.move_forward_client.get_result()
        if result and result.success:
            return 'succeeded'
        else:
            return 'failed'


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             output_keys=['enter_door_result'])
        self.result = result

    def execute(self, userdata):
        result = EnterDoorResult()
        result.success = self.result
        userdata.enter_door_result = result
        return 'succeeded'

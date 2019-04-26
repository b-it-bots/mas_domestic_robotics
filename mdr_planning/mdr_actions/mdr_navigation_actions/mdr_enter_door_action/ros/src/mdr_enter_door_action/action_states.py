#!/usr/bin/python
import rospy
import actionlib
from std_msgs.msg import Bool

from pyftsm.ftsm import FTSMTransitions
from mas_execution.action_sm_base import ActionSMBase
from mdr_move_forward_action.msg import MoveForwardAction, MoveForwardGoal
from mdr_enter_door_action.msg import EnterDoorFeedback, EnterDoorResult

class EnterDoorSM(ActionSMBase):
    def __init__(self, waiting_sleep_duration=1.,
                 door_status_topic='/mcr_perception/door_status/door_status',
                 timeout=120.0,
                 move_forward_server='move_forward_server',
                 movement_duration=10., speed=0.1,
                 max_recovery_attempts=1):
        super(EnterDoorSM, self).__init__('EnterDoor', [], max_recovery_attempts)

        self.waiting_sleep_duration = waiting_sleep_duration
        self.door_status_sub = rospy.Subscriber(door_status_topic, Bool,
                                                self.update_door_status)
        self.timeout = timeout
        self.movement_duration = movement_duration
        self.speed = speed
        self.move_forward_server = move_forward_server

        self.door_open = False
        self.move_forward_client = None

    def init(self):
        try:
            self.move_forward_client = actionlib.SimpleActionClient(self.move_forward_server, MoveForwardAction)
            rospy.loginfo('[enter_door] Waiting for %s server', self.move_forward_server)
            self.move_forward_client.wait_for_server()
        except:
            rospy.logerr('[enter_door] %s server does not seem to respond', self.move_forward_server)
        return FTSMTransitions.INITIALISED

    def running(self):
        rospy.loginfo('[enter_door] Waiting for door to open')
        while not self.door_open:
            rospy.sleep(self.waiting_sleep_duration)
        rospy.loginfo('[enter_door] Door open; entering')

        goal = MoveForwardGoal()
        goal.movement_duration = self.movement_duration
        goal.speed = self.speed

        self.move_forward_client.send_goal(goal)
        timeout_duration = rospy.Duration.from_sec(self.timeout)
        self.move_forward_client.wait_for_result(timeout_duration)
        result = self.move_forward_client.get_result()
        if result and result.success:
            self.result = self.set_result(True)
        self.result = self.set_result(False)
        return FTSMTransitions.DONE

    def update_door_status(self, msg):
        self.door_open = msg.data

    def set_result(self, success):
        result = EnterDoorResult()
        result.success = success
        return result

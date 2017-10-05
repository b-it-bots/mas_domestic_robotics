#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib

from std_msgs.msg import String

from mdr_enter_door_action.msg import EnterDoorAction, EnterDoorGoal
from mdr_introduce_self_action.msg import (IntroduceSelfAction,
                                           IntroduceSelfGoal)
from mdr_move_base_safe.msg import MoveBaseSafeAction, MoveBaseSafeGoal
from mdr_move_tray_action.msg import MoveTrayAction, MoveTrayGoal


class Enter(smach.State):
    def __init__(self, timeout=120.,
                 enter_action_server='/mdr_actions/enter_door_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             output_keys=['_feedback'])
        self.timeout = timeout
        self.enter_action_client = actionlib.SimpleActionClient(enter_action_server, EnterDoorAction)
        self.enter_action_client.wait_for_server()

    def execute(self, userdata):
        goal = EnterDoorGoal()

        rospy.loginfo('Calling enter_door action')
        self.enter_action_client.send_goal(goal)
        result = self.enter_action_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        rospy.loginfo('enter_door call completed')

        if result:
            return 'succeeded'
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
            goal.destination_location = destination_location

            rospy.loginfo('Sending the base to %s' % destination_location)
            self.client.send_goal(goal)
            self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))

            res = self.client.get_result()
            if res and res.success:
                rospy.loginfo('Successfully reached %s' % destination_location)
            else:
                rospy.logerr('Could not reach %s' % destination_location)
                return 'failed'
        return 'succeeded'


class IntroduceSelf(smach.State):
    def __init__(self, profession=True, residence=True, date_of_birth=True,
                 timeout=120.0, action_server='/mdr_actions/introduce_self_action_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.profession = profession
        self.residence = residence
        self.date_of_birth = date_of_birth
        self.timeout = timeout

        self.introduce_self_client = actionlib.SimpleActionClient(action_server, IntroduceSelfAction)
        self.introduce_self_client.wait_for_server()

    def execute(self, userdata):
        goal = IntroduceSelfGoal()
        goal.profession = self.profession
        goal.residence = self.residence
        goal.date_of_birth = self.date_of_birth

        rospy.loginfo('Calling introduce_self action')
        self.introduce_self_client.send_goal(goal)
        self.introduce_self_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        rospy.loginfo('introduce_self call completed')

        res = self.introduce_self_client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'


class Acknowledge(smach.State):
    def __init__(self, timeout=120.0, say_topic='/sound/say'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'], input_keys=['command'])
        self.timeout = timeout
        self.sound_pub = rospy.Publisher(say_topic, String, queue_size=1, latch=True)

    def execute(self, userdata):
        command = userdata.command.lower()
        publish_command = True
        msg = String()
        if 'move' in command:
            if 'up' in command:
                msg.data = 'Moving the tray up'
            elif 'down' in command:
                msg.data = 'Moving the tray down'
            else:
                rospy.logerr('Received unknown move tray direction')
                publish_command = False
        elif 'qr' in command:
            if 'continue' in command:
                msg.data = 'Received continue command from QR code'
            else:
                rospy.logerr('Received unkonwn QR code command')
                publish_command = False

        if publish_command:
            self.sound_pub.publish(msg)
            return 'succeeded'
        else:
            return 'failed'


class MoveTray(smach.State):
    def __init__(self, direction, timeout=120.0, action_server='/mdr_actions/move_tray_server'):
        smach.State.__init__(self, outcomes=['up', 'down', 'failed'], input_keys=['command'])
        self.timeout = timeout
        self.move_tray_client = actionlib.SimpleActionClient(action_server, MoveTrayAction)
        self.move_tray_client.wait_for_server()

    def execute(self, userdata):
        direction = None
        command = userdata.command
        if 'up' in command:
            direction = 'up'
        elif 'down' in command:
            direction = 'down'
        else:
            rospy.logerr('Received unknown move tray direction')
            return 'failed'

        goal = MoveTrayGoal()
        goal.direction = direction

        rospy.loginfo('Calling move_tray action with direction %s' % direction)
        self.move_tray_client.send_goal(goal)
        self.move_tray_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        rospy.loginfo('move_tray call completed')

        res = self.move_tray_client.get_result()
        if res and res.success:
            return direction
        else:
            return 'failed'


class WaitForQR(smach.State):
    def __init__(self, timeout=120.0, qr_topic='/mcr_perception/qr_reader/output'):
        smach.State.__init__(self, outcomes=['succeeded', 'waiting', 'failed'], output_keys=['command'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.state_check_rate = rospy.Rate(5)
        self.qr_code_sub = rospy.Subscriber(qr_topic, String, self.register_qr_code)
        self.qr_message = None
        self.start_time = rospy.Time.now()

    def execute(self, userdata):
        if (rospy.Time.now() - self.start_time) < self.timeout:
            if self.qr_message and self.qr_message.lower().find('continue') != -1:
                rospy.loginfo('QR message: %s' % self.qr_message)
                userdata.command = 'QR code message received: ' + self.qr_message
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


class WaitForCmd(smach.State):
    def __init__(self, timeout=120.0, speech_topic='/recognized_speech'):
        smach.State.__init__(self, outcomes=['succeeded', 'waiting', 'failed'], output_keys=['command'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.state_check_rate = rospy.Rate(5)
        self.speech_sub = rospy.Subscriber(speech_topic, String, self.command_cb)
        self.command = None
        self.start_time = rospy.Time.now()

    def execute(self, userdata):
        if (rospy.Time.now() - self.start_time) < self.timeout:
            if self.command:
                rospy.loginfo('Received command: %s' % self.command)
                userdata.command = self.command
                return 'succeeded'
            else:
                self.state_check_rate.sleep()
                rospy.loginfo('Waiting for command')
                return 'waiting'
        else:
            rospy.loginfo('wait_for_cmd timed out')
            return 'failed'

    def command_cb(self, msg):
        self.command = msg.data

#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib

from std_msgs.msg import String

from mdr_enter_door_action.msg import EnterDoorAction, EnterDoorGoal
from mdr_introduce_self_action.msg import IntroduceSelfAction, IntroduceSelfGoal
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

        self.introduce_self_client.send_goal(goal)
        self.introduce_self_client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

class Acknowledge(smach.State):
    def __init__(self, timeout=120.0, say_topic='/sound/say'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.message = message
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
                publish_command = False
        elif 'qr' in command:
            if 'continue' in command:
                msg.data = 'Received continue command from QR code'
            else:
                publish_command = False

        if publish_command:
            self.sound_pub.publish(msg)
            return 'succeeded'
        else:
            return 'failed'

class MoveTray(smach.State):
    def __init__(self, direction, timeout=120.0, action_server='/mdr_actions/move_tray_server'):
        smach.State.__init__(self, outcomes=['up', 'down', 'failed'])
        self.timeout = timeout

        self.move_tray_client = actionlib.SimpleActionClient(action_server, MoveTrayAction)
        self.move_tray_client.wait_for_server()

    def execute(self, userdata):
        goal = MoveTrayGoal()
        goal.direction = userdata.command

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

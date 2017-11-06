#!/usr/bin/python

import rospy
import smach
import smach_ros
import actionlib
import actionlib_msgs
from std_srvs.srv import Empty
from std_msgs.msg import String

import simple_script_server

import mdr_move_base_safe.msg
from mdr_move_base_safe.msg import MoveBaseSafeFeedback, MoveBaseSafeResult

class SetupMoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['move_base_safe_goal'],
                             output_keys=['move_base_safe_feedback', 'move_base_safe_result', 'pose'])

    def execute(self, userdata):
        # get base goal from actionlib
        base_goal = userdata.move_base_safe_goal.destination_location
        userdata.pose = base_goal

        feedback = MoveBaseSafeFeedback()
        feedback.current_state = 'MOVE_BASE'
        feedback.text = '[move_base_safe] moving the base to ' + base_goal
        userdata.move_base_safe_feedback = feedback

        return 'succeeded'

class ClearCostmaps(smach.State):
    def __init__(self, timeout=120., base_recover_srv_name='/move_base/clear_costmaps'):
        smach.State.__init__(self, outcomes=['succeeded','failed'])
        self.timeout = timeout
        self.base_recover_srv_name = base_recover_srv_name
        self.base_recover_srv = rospy.ServiceProxy(self.base_recover_srv_name, Empty)

    def execute(self, userdata):
        rospy.wait_for_service(self.base_recover_srv_name, self.timeout)
        try:
            self.base_recover_srv()
            return 'succeeded'
        except rospy.ServiceException, e:
            print("Service call to {0} failed: {1}".format(self.base_recover_srv_name, e))
            return 'failed'

class ApproachPose(smach.State):
    def __init__(self, pose='', mode='omni'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'reset'],
                             input_keys=['pose', 'message'],
                             output_keys=['pose', 'message'])
        self.pose = pose
        self.mode = mode
        self.speak_pub = rospy.Publisher('/sound/say', String, latch=True)

    def execute(self, userdata):
        # determine target position
        if self.pose != '':
            pose = self.pose
        elif type(userdata.pose) is str:
            pose = userdata.pose
        elif type(userdata.pose) is list:
            pose = []
            pose.append(userdata.pose[0])
            pose.append(userdata.pose[1])
            pose.append(userdata.pose[2])
        else: # this should never happen
            userdata.message = []
            userdata.message.append(5)
            userdata.message.append('Invalid userdata "pose"')
            userdata.message.append(userdata.pose)
            return 'failed'

        # try reaching pose
        cob_script_server = simple_script_server.simple_script_server()
        handle_base = cob_script_server.move('base', pose, False, self.mode)
        move_second = False

        timeout = 0
        while True:
            move_base_state = handle_base.get_state()
            if (move_base_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED) and (not move_second):
                # do a second movement to place the robot more exactly
                handle_base = cob_script_server.move('base', pose, False, self.mode)
                move_second = True
            elif (move_base_state == actionlib_msgs.msg.GoalStatus.SUCCEEDED) and (move_second):
                userdata.message = []
                userdata.message.append(3)
                userdata.message.append('Pose was succesfully reached')
                return 'succeeded'
            elif move_base_state == actionlib_msgs.msg.GoalStatus.REJECTED or move_base_state == actionlib_msgs.msg.GoalStatus.PREEMPTED or move_base_state == actionlib_msgs.msg.GoalStatus.ABORTED or move_base_state == actionlib_msgs.msg.GoalStatus.LOST:
                self.speak_pub.publish('I can not reach my target position.')
                rospy.logerr('base movement failed with state: %d', move_base_state)
                return 'failed'
            rospy.sleep(0.1)

class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['move_base_safe_goal'],
                             output_keys=['move_base_safe_feedback', 'move_base_safe_result'])
        self.result = result

    def execute(self, userdata):
        result = MoveBaseSafeResult()
        result.success = self.result
        userdata.move_base_safe_result = result
        return 'succeeded'

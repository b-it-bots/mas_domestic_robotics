#!/usr/bin/python

import rospy
import smach
import smach_ros
import std_msgs.msg
import actionlib
import mdr_actions.msg


class move_base_safe(smach.State):
    def __init__(self, destination_location, timeout=120.0, action_server='move_base_safe_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.destination_location = destination_location
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.MoveBaseSafeAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.MoveBaseSafeGoal()
        goal.source_location = 'anywhere'
        goal.destination_location = self.destination_location
        #rospy.loginfo('Sending actionlib goal to ' + self.action_server + ', destination: ',
        #              goal.destination_location + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'


class perceive_shelf(smach.State):
    def __init__(self, timeout=30.0, action_server='perceive_shelf_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.PerceiveShelfAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.PerceiveShelfGoal()
        goal.location = 'anywhere'
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'


class pick_object(smach.State):
    def __init__(self, timeout=30.0, action_server='pick_object_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.PickObjectAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.PickObjectGoal()
        goal.object = 'anything'
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'


class place_object(smach.State):
    def __init__(self, timeout=30.0, action_server='place_object_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.PlaceObjectAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.PlaceObjectGoal()
        goal.object = 'anything'
        goal.location = 'anywhere'
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'


class recognize_objects(smach.State):
    def __init__(self, timeout=15.0, action_server='recognized_objects_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.RecognizeObjectsAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.RecognizeObjectsGoal()
        goal.location = 'anywhere'
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

class turn_and_answer(smach.State):
    def __init__(self, timeout=45.0, action_server='turn_and_answer_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.TurnAndAnswerAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.TurnAndAnswerGoal()
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

class answer_question(smach.State):
    def __init__(self, timeout=15.0, action_server='question_answer_server'):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.action_server = action_server
        self.client = actionlib.SimpleActionClient(action_server, mdr_actions.msg.QuestionAnswerAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        goal = mdr_actions.msg.QuestionAnswerGoal()
        rospy.loginfo('Sending actionlib goal to ' + self.action_server + ' with timeout: ' + str(self.timeout))
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration.from_sec(self.timeout))
        res = self.client.get_result()
        if res and res.success:
            return 'succeeded'
        else:
            return 'failed'

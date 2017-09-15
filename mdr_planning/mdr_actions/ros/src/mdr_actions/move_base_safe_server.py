#!/usr/bin/env python
import rospy
import smach

import mdr_common_states.common_states_navigation as gns  # move the base

# action lib
from smach_ros import ActionServerWrapper
from mdr_actions.msg import MoveBaseSafeAction
from mdr_actions.msg import MoveBaseSafeFeedback
from mdr_actions.msg import MoveBaseSafeResult


class SetupMoveBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'],
                             input_keys=['move_base_safe_goal'],
                             output_keys=['move_base_safe_feedback', 'move_base_safe_result', 'pose'])

    def execute(self, userdata):
        # get base goal from actionlib
        base_goal = userdata.move_base_safe_goal.destination_location
        feedback = MoveBaseSafeFeedback()
        feedback.current_state = 'MOVE_BASE'
        feedback.text = '[move_base_safe] moving the base to ' + base_goal
        userdata.move_base_safe_feedback = feedback
        userdata.pose = base_goal
        return 'succeeded'


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


# ===============================================================================

def main():
    rospy.init_node('move_base_safe_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILED'],
            input_keys=['move_base_safe_goal'],
            output_keys=['move_base_safe_feedback', 'move_base_safe_result'])
    with sm:
        smach.StateMachine.add('SETUP_MOVE_BASE', SetupMoveBase(),
                               transitions={'succeeded': 'MOVE_BASE_TO_DESTINATION',
                                            'failed': 'SETUP_MOVE_BASE'})

        smach.StateMachine.add('MOVE_BASE_TO_DESTINATION', gns.approach_pose(),
                               transitions={'success': 'SET_ACTION_LIB_SUCCESS',
                                            'failed': 'SET_ACTION_LIB_FAILED'})

        smach.StateMachine.add('SET_ACTION_LIB_FAILED', SetActionLibResult(False),
                               transitions={'succeeded': 'OVERALL_FAILED'})

        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                               transitions={'succeeded': 'OVERALL_SUCCESS'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name='move_base_safe_server',
        action_spec=MoveBaseSafeAction,
        wrapped_container=sm,
        succeeded_outcomes=['OVERALL_SUCCESS'],
        aborted_outcomes=['OVERALL_FAILED'],
        preempted_outcomes=['PREEMPTED'],
        goal_key='move_base_safe_goal',
        feedback_key='move_base_safe_feedback',
        result_key='move_base_safe_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()

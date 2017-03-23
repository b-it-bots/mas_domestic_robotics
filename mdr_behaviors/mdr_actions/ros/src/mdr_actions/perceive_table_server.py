#!/usr/bin/env python
import rospy
import smach

import mdr_common_states.common_states_manipulation as gms
import mcr_states.common.perception_states as gps
import mcr_states.common.basic_states as gbs

# action lib
from smach_ros import ActionServerWrapper
from mdr_actions.msg import PerceiveTableAction
from mdr_actions.msg import PerceiveTableFeedback
from mdr_actions.msg import PerceiveTableResult


class SetActionLibResult(smach.State):
    def __init__(self, result):
        smach.State.__init__(self, outcomes=['succeeded'],
                             input_keys=['perceive_table_goal'],
                             output_keys=['perceive_table_feedback', 'perceive_table_result'])
        self.result = result

    def execute(self, userdata):
        result = PerceiveTableResult()
        result.success = self.result
        userdata.perceive_table_result = result
        return 'succeeded'


def main():
    rospy.init_node('perceive_table_server')
    # Construct state machine
    sm = smach.StateMachine(
            outcomes=['OVERALL_SUCCESS', 'OVERALL_FAILED'],
            input_keys=['perceive_table_goal'],
            output_keys=['perceive_table_feedback', 'perceive_table_result'])
    with sm:
        smach.StateMachine.add('MOVE_ARM', gms.move_arm('look_at_table_front'),
                               transitions={'succeeded': 'MOVE_TORSO_BACK',
                                            'failed': 'MOVE_TORSO_FRONT'})

        smach.StateMachine.add('MOVE_TORSO_FRONT', gms.move_part('torso', 'front'),
                               transitions={'succeeded': 'MOVE_ARM',
                                            'failed': 'MOVE_TORSO_FRONT'})

        smach.StateMachine.add('MOVE_TORSO_BACK', gms.move_part('torso', 'back_extreme'),
                               transitions={'succeeded': 'MOVE_HEAD_BACK',
                                            'failed': 'MOVE_TORSO_BACK'})

        smach.StateMachine.add('MOVE_HEAD_BACK', gms.move_part('head', 'back_table'),
                               transitions={'succeeded': 'CONFIGURE_TABLE',
                                            'failed': 'MOVE_HEAD_BACK'})

        smach.StateMachine.add('CONFIGURE_TABLE', gbs.set_named_config('table'),
                               transitions={'success': 'START_WORKSPACE_FINDER',
                                            'failure': 'SET_ACTION_LIB_FAILURE',
                                            'timeout': 'CONFIGURE_TABLE'})

        smach.StateMachine.add('START_WORKSPACE_FINDER', gbs.send_event(
                event_list=[('/mcr_perception/mux_pointcloud/select', '/cam3d/depth_registered/points')]),
                transitions={'success': 'RECOGNIZE_OBJECTS'})

        smach.StateMachine.add('RECOGNIZE_OBJECTS', gps.find_objects(retries=1),
                               transitions={'objects_found': 'SET_ACTION_LIB_SUCCESS',
                                            'no_objects_found': 'SET_ACTION_LIB_FAILURE'},
                               remapping={'found_objects': 'recognized_objects'})

        smach.StateMachine.add('SET_ACTION_LIB_SUCCESS', SetActionLibResult(True),
                               transitions={'succeeded': 'OVERALL_SUCCESS'})

        smach.StateMachine.add('SET_ACTION_LIB_FAILURE', SetActionLibResult(False),
                               transitions={'succeeded': 'OVERALL_FAILED'})

    # Construct action server wrapper
    asw = ActionServerWrapper(
        server_name='perceive_table_server',
        action_spec=PerceiveTableAction,
        wrapped_container=sm,
        succeeded_outcomes=['OVERALL_SUCCESS'],
        aborted_outcomes=['OVERALL_FAILED'],
        preempted_outcomes=['PREEMPTED'],
        goal_key='perceive_table_goal',
        feedback_key='perceive_table_feedback',
        result_key='perceive_table_result')
    # Run the server in a background thread
    asw.run_server()
    rospy.spin()

if __name__ == '__main__':
    main()

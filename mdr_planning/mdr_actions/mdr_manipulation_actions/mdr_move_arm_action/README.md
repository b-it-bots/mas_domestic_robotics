# mdr_move_arm_action

An action for moving the arm of a robot.

## Action definition

### Goal constants:

#### Named targets:

* ``string HOME=home``
* ``string FOLDED=folded``
* ``string PREGRASP=pregrasp``
* ``string PREGRASP_TOP=pregrasp_top``
* ``string GRASP=grasp``
* ``string HOLD=hold``
* ``string PRE_LOOK_AT_TABLE=pre_look_at_table``
* ``string LOOK_AT_TABLE=look_at_table``

#### Goal types:

* ``int32 NAMED_TARGET=1``
* ``int32 END_EFFECTOR_POSE=2``
* ``int32 JOINT_VALUES=3``

### Goal:

* ``int32 goal_type``: The type of motion goal (one of the allowed goal types defined above)
* ``string named_target``: Motion goal if ``goal_type`` is ``1``
* ``geometry_msgs/PoseStamped end_effector_pose``: Motion goal if ``goal_type`` is ``2``
* ``float64[] joint_values``: Motion goal if ``goal_type`` is ``3``

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Dependencies

* ``geometry_msgs``
* ``moveit_commander``

## Example usage

1. Run the robot simulation: ``roslaunch mdr_bringup_sim robot.launch``
2. Run the cob moveit interface: ``roslaunch mdr_moveit_cob move_group.launch``
3. Run the action server: ``roslaunch mdr_move_arm_action move_arm.launch``
4. Run the client example
    1. with a named target motion goal: ``rosrun mdr_move_arm_action move_arm_action_client_test 1 folded``
    2. with a pose motion goal: ``rosrun mdr_move_arm_action move_arm_action_client_test 2 "['base_link', 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]"``
    3. with a goal for the joint values: ``rosrun mdr_move_arm_action move_arm_action_client_test 3 "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"``

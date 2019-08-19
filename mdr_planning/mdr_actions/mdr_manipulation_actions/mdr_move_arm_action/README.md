# mdr_move_arm_action

An action for moving the arm of a robot.

The action uses MoveIt! for moving the arm to named targets and specified joint values; motion to arbitrary 3D poses can be performed either using a dynamic motion primitives or using MoveIt!.

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
* ``string named_target``: Motion goal if ``goal_type`` is ``NAMED_TARGET``
* ``geometry_msgs/PoseStamped end_effector_pose``: Motion goal if ``goal_type`` is ``END_EFFECTOR_POSE``
* ``string dmp_name``: Path to a YAML file containing the weights of a dynamic motion primitive if ``goal_type`` is ``END_EFFECTOR_POSE`` (if the value is an empty string, MoveIt! is used for planning a trajectory and moving the arm)
* ``float64 dmp_tau``: The value of the temporal dynamic motion primitive parameter if ``goal_type`` is ``END_EFFECTOR_POSE``
* ``float64[] joint_values``: Motion goal if ``goal_type`` is ``JOINT_VALUES``

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Directory structure

```
mdr_move_arm_action
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____action
     |    |_____MoveArm.action
     |    |
     |____launch
     |    |_____move_arm.launch
     |    |
     |    scripts
     |    |     move_arm_action
     |    |_____move_arm_action_client_test
     |    |
     |____src
          |____mdr_move_arm_action
               |    __init__.py
               |    action_states.py
               |____dmp.py
```

## Dependencies

* ``smach``
* ``geometry_msgs``
* ``moveit_commander``

## Example usage

1. Run the robot simulation: ``roslaunch mas_<robot>_bringup_sim robot.launch``
2. Run the moveit interface for the robot: ``roslaunch mas_<robot>_moveit move_group.launch``
3. Run the action server: ``roslaunch mdr_move_arm_action move_arm.launch``
4. Run the client example:
    1. with a named target motion goal: ``rosrun mdr_move_arm_action move_arm_action_client_test 1 folded``
    2. with a pose motion goal: ``rosrun mdr_move_arm_action move_arm_action_client_test 2 "['base_link', <x, y, z>, <x, y, z, w>]"``
    3. with a goal for the joint values: ``rosrun mdr_move_arm_action move_arm_action_client_test 3 "[<joint values>]"``

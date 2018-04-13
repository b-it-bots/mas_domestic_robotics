# mdr_turn_base_to_action

An action to turn a robot base to face a given orientation

## Action definition

### Goal:
* ``float64 desired_yaw``: Yaw Angle in radians that the robot will face to

### Result:

* ``bool success``

### Feedback:

* ``string message``

## Directory structure

```
.
├── CMakeLists.txt
├── package.xml
├── README.md
├── ros
│   ├── action
│   │   └── TurnBaseTo.action
│   ├── launch
│   │   └── turn_base_to_server.launch
│   ├── scripts
│   │   ├── turn_base_to_action
│   │   └── turn_base_to_action_client_test
│   └── src
│       └── mdr_turn_base_to_action
│           ├── action_states.py
│           ├── action_states.pyc
│           └── __init__.py
└── setup.py

```

## Launch file
* ``turn_base_to_server_solo.launch`` : Run mdr_turn_base_to_action and mdr_move_base_action
* ``turn_base_to_server.launch`` : Run just the mdr_turn_base_to_action

## Launch file parameters
* ``move_base_server``: Name of the mdr `move_base` action server (default: '/move_base_server')
* ``rotation_frame``:  Name of the frame in which the robot will rotate according to

### Action server

The following parameters may be passed when launching the action server:
* ``move_base_server``: Name of the mdr `move_base` action server (default: '/move_base_server')
* ``rotation_frame``:  Name of the frame in which the robot will rotate according to

### Action client

The following parameters need to be passed when launching the action client:
* ``desired_yaw``: desired yaw rotation in radians

## Action execution summary

This action is an extension of the `mdr_move_base_action` and thus sends a desired rotation according a given frame.

## Dependencies

* ``smach``
* ``smach_ros``
* ``tf``
* ``actionlib``
* ``geometry_msgs``
* ``mdr_move_base_action``

## Example usage

### Test Client
1. Run the robot simulation: ``roslaunch mas_<robot>_bringup_sim robot.launch``
2. Run the move_base_action server: ``roslaunch mdr_move_base_action move_base_action.launch``
2. Run the turn_base_to_action server: ``roslaunch mdr_turn_base_to_action turn_base_to_action.launch``

5. Run the client example: ``rosrun mdr_turn_base_to_action turn_base_to_action_client_test <DESIRED_YAW>``; the client example only allows using the action with a named target

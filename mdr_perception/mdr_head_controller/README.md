# mdr_head_controller

A package for functionalities related to controlling a robot's head.

## Existing Functionalities

### head_controller_base

A base implementation of a high-level head controller for performing basic head motion commands, such as tilting the head up or down.

The base class defines the following methods for controlling a head:
* `look_up`: Moves the robot's head up. Returns a Boolean indicating whether the operation was successful.
* `look_down`: Moves the robot's head down. Returns a Boolean indicating whether the operation was successful.
* `turn_left`: Turns the robot's head to the left. Returns a Boolean indicating whether the operation was successful.
* `turn_right`: Turns the robot's head to the right. Returns a Boolean indicating whether the operation was successful.
* `tilt`: tilt the head (vertically) by a specific angle.
* `pan`: pan the head (horizontally) by a specific angle.
* `reset`: reset the head to an initial position

Robot-specific implementations need to override all methods.

A script that starts a `head_controller` node is also included in the package as an example, but a robot-specific implementation should start its own node.

## Directory structure

```
mdr_head_controller
├── CMakeLists.txt
├── package.xml
├── README.md
├── ros
│   ├── launch
│   │   └── head_controller.launch
│   ├── scripts
│   │   └── head_controller
│   └── src
│       └── mdr_head_controller
│           ├── head_controller_base.py
│           ├── head_controller.py
│           └── __init__.py
└── setup.py
```

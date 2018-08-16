# mdr_command_robot

A package for functionalities related to processing robot commands.

## Existing Functionalities

### spoken_joypad_base

A base implementation of a "spoken joypad" that allows moving a robot using spoken commands. In order for a command to trigger motion, it should contain three elements: (i) the name of a robot, (ii) a keyword specifying the type of motion that should be performed, and (iii) a direction keyword. All other parts of the command are ignored.

The base class defines and implements the following methods for reacting to motion commands:
* `parse_command`: Processes a spoken string and calls a method for sending commands based on the type of motion
* `move_base`: Sends a linear motion command based on a direction keyword (`forward`, `backward`, `left`, or `right`)
* `turn_base`: Sends a rotational motion command based on a direction keyword (`left` or `right`)
* `move_head`: The base class only has a stub implementation that ignores all requests

The base class implementations of `parse_command`, `move_head`, and `turn base` are robot-independent, so robot-specific implementations may only need to override `move_head`.

A script that starts a `spoken_joypad` node is also included in the package as an example, but a robot-specific implementation should start its own node.

## Launch file parameters

### spoken_joypad

The following parameters may be passed when launching the spoken joypad:
* ``robot_name``: A string specifying the name of a robot
* ``recognized_speech_topic``: The name of a topic on which recognized speech is published
* ``base_vel_topic``: The name of a topic for sending velocity commands
* ``base_linear_vel``: Linear robot velocity in m/s
* ``base_angular_vel``: Angular robot velocity in rad/s
* ``move_base_keywords``: A list of keywords to which a robot should react for performing linear motion (e.g. [go, move])
* ``turn_base_keywords``: A list of keywords to which a robot should react for performing rotational motion (e.g. [turn])
* ``move_head_keywords``: A list of keywords to which a robot should react for performing head motion (e.g. [look])

## Directory structure

```
mdr_command_robot
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____launch
     |    |_____spoken_joypad.launch
     |    |
     |    scripts
     |    |_____spoken_joypad
     |    |
     |____src
          |____mdr_command_robot
               |    __init__.py
               |____spoken_joypad_base.py
```
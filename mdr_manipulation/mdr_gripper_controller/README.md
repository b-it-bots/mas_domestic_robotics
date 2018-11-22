# mdr_gripper_controller

A package for functionalities related to controlling a robot's gripper.

## Existing Functionalities

### gripper_controller_base

A base implementation of a high-level gripper controller for performing basic gripper commands, such as opening and closing a gripper.

The base class defines the following methods for controlling a gripper:
* `open`: Opens a robot's gripper. Returns a Boolean indicating whether the operation was successful.
* `close`: Closes a robot's gripper. Returns a Boolean indicating whether the operation was successful.
* `init_grasp_verification`: Performs any necessary initialisation procedures for verifying grasps (e.g. pre-grasp force measurements)
* `verify_grasp`: Returns a Boolean indicating whether there is an object in the gripper.

Robot-specific implementations need to override all methods.

A script that starts a `gripper_controller` node is also included in the package as an example, but a robot-specific implementation should start its own node.

## Directory structure

```
mdr_gripper_controller
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____launch
     |    |_____gripper_controller.launch
     |    |
     |    scripts
     |    |_____gripper_controller
     |    |
     |____src
          |____mdr_gripper_controller
               |    __init__.py
               |____gripper_controller_base.py
```

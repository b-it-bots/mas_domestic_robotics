# mdr_push_pull_object_action

An action for pushing or pulling an object to a predefined goal region.

## Action definition

### Goal:

* ``geometry_msgs/PoseStamped object_pose``: Pose of the object to be pushed
* ``geometry_msgs/PoseStamped goal_pose``: Pose to which the object should be pushed
* ``float32 goal_distance_tolerance_m``: Goal tolerance in meters; this defines a goal region (currently unused)
* ``string context``: Purpose for which the action is performed

The following constants are also defined in the action goal:
* ``string CONTEXT_MOVING=push_pull_to_move``
* ``string CONTEXT_TABLETOP_MANIPULATION=push_pull_for_tabletop_manipulation``

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Directory structure

```
mdr_push_pull_object_action
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____action
     |    |_____PushPullObject.action
     |    |
     |____launch
     |    |_____push_pull_object.launch
     |    |
     |    scripts
     |    |     push_pull_object_action
     |    |_____push_pull_object_action_client_test
     |    |
     |____src
          |____mdr_push_pull_object_action
               |    __init__.py
               |____action_states.py
```

## Launch file parameters

### Action server

The following parameters may be passed when launching the action server:
* ``gripper_controller_pkg_name``: The name of a package that implements functionalities for controlling a robot's gripper (default: 'mdr_gripper_controller')
* ``move_arm_server``: Name of the `move_arm` action server (default: 'move_arm_server')
* ``cmd_vel_topic``: Name of a topic on which base velocity commands can be sent (default '/cmd_vel')
* ``movement_speed_ms``: Base movement speed during pushing (default: 0.1)
* ``safe_arm_joint_config``: The name of a configuration in which the robot can safely move around the environment (default: 'folded')
* ``number_of_retries``: Number of times a grasp should be repeated in case it fails the first time.

## Action execution summary

The action performs pushing with respect to the `base_link` frame (even if the goal pose is expressed in another frame) and is executed in a few steps:
1. The object is pushed by moving the base towards the goal
2. If the context is not `CONTEXT_TABLETOP_MANIPULATION`, the arm is retrieved back to a folded configuration and then the base is moved back to the position from which it started pushing

## Dependencies

* ``tf``
* ``actionlib``
* ``geometry_msgs``
* ``mas_execution_manager``
* ``mdr_move_arm_action``
* ``mdr_move_base_action``

## Example usage

1. Run the ``move_arm`` action server: ``roslaunch mas_<robot>_move_arm_action move_arm.launch``
2. Run the action server: ``roslaunch mas_<robot>_push_pull_action push.launch``
3. Run the client example: ``rosrun mdr_push_pull_object_action push_pull_action_client_test``; the client example uses predefined object and goal poses

# mdr_push_pull_object_action

An action for pushing or pulling an object to a predefined goal region.

## Action definition

### Goal:

* ``geometry_msgs/PoseStamped object_pose``: Pose of the object to be pushed
* ``geometry_msgs/PoseStamped goal_pose``: Pose to which the object should be pushed
* ``float32 goal_distance_tolerance_m``: Goal tolerance in meters; this defines a goal region (currently unused)

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
* ``pregrasp_config_name``: The name of the default pregrasp configuration for side grasps (default: 'pregrasp')
* ``pregrasp_low_config_name``: The name of a pregrasp configuration for low sideways grasps (default: 'pregrasp_low')
* ``pregrasp_height_threshold``: Height threshold for selecting high or low pregrasp poses (default: 0.5)
* ``move_arm_server``: Name of the `move_arm` action server (default: 'move_arm_server')
* ``move_base_server``: Name of the `move_base` action server (default: 'move_base_server')
* ``cmd_vel_topic``: Name of a topic on which base velocity commands can be sent (default '/cmd_vel')
* ``movement_speed_ms``: Base movement speed during pushing (default: 0.1)
* ``safe_arm_joint_config``: The name of a configuration in which the robot can safely move around the environment (default: 'folded')
* ``grasping_orientation``: For more constrained manipulators, it might make sense to use a fixed grasping orientation (expressed as an (x, y, z, w) quaternion) to ensure easier reachability (default: [], in which case the argument is ignored)
* ``base_elbow_offset``: An optional offset between `base_link` and the manipulator's elbow; used for aligning the base with the object to be grasped so that the manipulator can easily reach the object (default: -1)
* ``grasping_dmp``:  Path to a YAML file containing the weights of a dynamic motion primitive used for grasping (default: '')
* ``dmp_tau``: The value of the temporal dynamic motion primitive parameter (default: 1)
* ``number_of_retries``: Number of times a grasp should be repeated in case it fails the first time.

## Action execution summary

The action performs grasping and then pushing with respect to the `base_link` frame (even if the goal pose is expressed in another frame) and is executed in a few steps:
1. If ``base_elbow_offset`` is greater than 0, the base is aligned with the object so that the origin of `base_link` is ``base_elbow_offset`` units away from the object's pose
2. The manipulator is moved to a predefined pregrasp configuration
3. The end-effector is then sent to its grasping goal and the gripper is closed; if a path to a dynamic motion primitive file is passed as a parameter to the action, the grasping trajectory is represented by the motion primitive
4. The object is pushed by moving the base towards the goal
5. The arm is retrieved back to a folded configuration and then the base is moved back to the position from which it started pushing

## Dependencies

* ``tf``
* ``actionlib``
* ``geometry_msgs``
* ``mas_execution_manager``
* ``mdr_move_arm_action``
* ``mdr_move_base_action``

## Example usage

1. Run the ``move_arm`` action server: ``roslaunch mas_<robot>_move_arm_action move_arm.launch``
2. Run the ``move_base`` action server: ``roslaunch mas_<robot>_move_base_action move_base.launch``
3. Run the action server: ``roslaunch mas_<robot>_push_action push.launch``
4. Run the client example: ``rosrun mdr_push_pull_object_action push_pull_action_client_test``; the client example uses predefined object and goal poses

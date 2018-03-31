# mdr_pickup_action

An action for moving the arm to a specified pose and closing the gripper around it.

The action includes an action server as well as an action client that interacts with ROSPlan.

## Action definition

### Goal:

* ``geometry_msgs/PoseStamped pose``: An end-effector goal pose
* ``float[] closed_gripper_joint_values``: A list of gripper joint values in which the gripper is considered closed (in principle, these are object- and context-dependent)

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Directory structure

```
mdr_pickup_action
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____action
     |    |_____Pickup.action
     |    |
     |____launch
     |    |     pickup_action.launch
     |    |_____pickup_client.launch
     |    |
     |    scripts
     |    |     pickup_action
     |    |     pickup_action_client_test
     |    |_____pickup_client
     |    |
     |____src
          |____mdr_pickup_action
               |    __init__.py
               |____action_states.py
```

## Launch file parameters

### Action server

The following parameters may be passed when launching the action server:
* ``arm_name``: The name of the arm moveit group (default: 'arm')
* ``gripper_cmd_topic``: The name of a topic that the gripper controller listens to for gripper commands (default: 'gripper_controller')
* ``gripper_joint_names``: A list of names of end-effector joints as expected by the gripper controller (default: ['gripper'])
* ``pregrasp_config_name``: The name of the pregrasp configuration specified in the moveit configuration (default: 'pregrasp')
* ``intermediate_grasp_offset``: An optional pose offset that creates an intermediate trajectory goal following the pregrasp configuration (default: -1)
* ``safe_arm_joint_config``: The name of a configuration specified in the moveit configuration in which the robot can safely move around the environment (default: 'folded')

### Action client

The action client that interacts with ROSPlan inherits from the action client base class defined in ``mdr_rosplan_interface``.

The following parameters need to be passed when launching the action client:
* ``action_name``: Name of the action as used in the planning domain
* ``server_name``: Name of the pickup action server
* ``action_timeout``: Maximum time (in seconds) that we are willing to wait for the action to be executed
* ``action_dispatch_topic``: Name of the topic at which the plan dispatcher sends action requests
* ``action_feedback_topic``: Name of the topic at which the action sends feedback to the plan dispatcher
* ``knowledge_update_service``: Name of a service used for updating the planning problem as the world changes
* ``attribute_fetching_service``: Name of a service used for retrieving attributes representing the current knowledge about the world
* ``grasping_pose_frame``: Name of the frame in which grasping is performed (default: 'base_link')
* ``closed_gripper_joint_values``: A list of gripper joint values at which the gripper is considered closed (in a general case, these should be determined by a grasp planner) (default: [0])

## Action execution summary

The action performs grasping with respect to the `base_link` frame (even if the goal pose is expressed in another frame) and is executed in a few steps:
1. The manipulator is moved to a predefined pregrasp configuration
2. If ``intermediate_grasp_offset`` is greater than 0, the end-effector is sent to an intermediate goal pose that is ``intermediate_grasp_offset`` meters away (along `base_link`'s x-axis) from the grasping goal
3. The end-effector is then sent to its grasping goal and the gripper is closed
4. The manipulator is moved to a configuration in which the robot can safely move around in the environment

## Dependencies

* ``smach``
* ``smach_ros``
* ``tf``
* ``actionlib``
* ``moveit_commander``
* ``geometry_msgs``
* ``trajectory_msgs``
* ``rosplan_dispatch_msgs``
* ``rosplan_dispatch_msgs``
* ``diagnostic_msgs``
* ``mcr_perception_msgs``
* ``mdr_rosplan_interface``
* ``mdr_move_arm_action``

## Example usage

1. Run the robot simulation: ``roslaunch mas_<robot>_bringup_sim robot.launch``
2. Run the moveit interface for the robot: ``roslaunch mas_<robot>_moveit move_group.launch``
3. Run the ``move_arm`` action server: ``roslaunch mdr_move_arm_action move_arm.launch``
4. Run the action server: ``roslaunch mdr_pickup_action pickup_action.launch``
5. Run the client example: ``rosrun mdr_pickup_action pickup_action_client_test``; the client example uses a predefined grasping pose

# mdr_pickup_action

An action for moving the arm to a specified pose and closing the gripper around it.

The action includes an action server as well as an action client that interacts with ROSPlan.

## Action definition

### Goal:

* ``geometry_msgs/PoseStamped pose``: An end-effector goal pose
* ``uint32 strategy``: Grasping strategy

The following constants are also defined in the action goal:
* ``uint32 SIDEWAYS_GRASP=0``
* ``uint32 TOP_GRASP=1``

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
* ``move_arm_server``: Name of the `move_arm` action server (default: 'move_arm_server')
* ``move_base_server``: Name of the `move_base` action server (default: 'move_base_server')
* ``gripper_controller_pkg_name``: The name of a package that implements functionalities for controlling a robot's gripper (default: 'mdr_gripper_controller')
* ``pregrasp_config_name``: The name of the default pregrasp configuration for side grasps (default: 'pregrasp')
* ``pregrasp_top_config_name``: The name of a pregrasp configuration for top grasps (default: 'pregrasp_top')
* ``pregrasp_low_config_name``: The name of a pregrasp configuration for low sideways grasps (default: 'pregrasp_low')
* ``pregrasp_height_threshold``: Height threshold for selecting high or low pregrasp poses (default: 0.5)
* ``intermediate_grasp_offset``: An optional pose offset that creates an intermediate trajectory goal following the pregrasp configuration (default: -1)
* ``safe_arm_joint_config``: The name of a configuration in which the robot can safely move around the environment (default: 'folded')
* ``base_elbow_offset``: An optional offset between `base_link` and the manipulator's elbow; used for aligning the base with the object to be grasped so that the manipulator can easily reach the object (default: -1)
* ``grasping_dmp``:  Path to a YAML file containing the weights of a dynamic motion primitive used for grasping (default: '')
* ``dmp_tau``: The value of the temporal dynamic motion primitive parameter (default: 1)
* ``grasping_orientation``: For more constrained manipulators, it might make sense to use a fixed grasping orientation (expressed as an (x, y, z, w) quaternion) to ensure easier reachability (default: [], in which case the argument is ignored)
* ``number_of_retries``: Number of times a grasp should be repeated in case it fails the first time.

### Action client

The action client that interacts with ROSPlan inherits from the action client base class defined in ``mdr_rosplan_interface``.

The following parameters need to be passed when launching the action client:
* ``action_name``: Name of the action as used in the planning domain
* ``server_name``: Name of the pickup action server
* ``action_timeout``: Maximum time (in seconds) that we are willing to wait for the action to be executed
* ``action_dispatch_topic``: Name of the topic at which the plan dispatcher sends action requests
* ``action_feedback_topic``: Name of the topic at which the action sends feedback to the plan dispatcher
* ``grasping_pose_frame``: Name of the frame in which grasping is performed (default: 'base_link')

## Action execution summary

The action performs grasping with respect to the `base_link` frame (even if the goal pose is expressed in another frame) and is executed in a few steps:
1. If ``base_elbow_offset`` is greater than 0, the base is aligned with the object so that the origin of `base_link` is ``base_elbow_offset`` units away from the object's pose
2. The manipulator is moved to a predefined pregrasp configuration
3. If ``intermediate_grasp_offset`` is greater than 0, the end-effector is sent to an intermediate goal pose that is ``intermediate_grasp_offset`` meters away (along `base_link`'s x-axis) from the grasping goal
4. The end-effector is then sent to its grasping goal and the gripper is closed; if a path to a dynamic motion primitive file is passed as a parameter to the action, the grasping trajectory is represented by the motion primitive
5. The manipulator is moved to a configuration in which the robot can safely move around in the environment

## Dependencies

* ``smach``
* ``smach_ros``
* ``tf``
* ``actionlib``
* ``geometry_msgs``
* ``trajectory_msgs``
* ``rosplan_dispatch_msgs``
* ``rosplan_knowledge_msgs``
* ``diagnostic_msgs``
* ``mas_perception_msgs``
* ``mdr_rosplan_interface``
* ``mdr_move_arm_action``
* ``mdr_move_base_action``

## Example usage

1. Run the robot simulation: ``roslaunch mas_<robot>_bringup_sim robot.launch``
3. Run the ``move_arm`` action server: ``roslaunch mas_<robot>_move_arm_action move_arm.launch``
4. Run the ``move_base`` action server: ``roslaunch mas_<robot>_move_base_action move_base.launch``
5. Run the action server: ``roslaunch mas_<robot>_pickup_action pickup_action.launch``
6. Run the client example: ``rosrun mdr_pickup_action pickup_action_client_test``; the client example uses a predefined grasping pose

Place Object Action
====================

mdr_place_action
-----------------

An action for moving the arm to a specified pose and opening the gripper there.
The action includes an action server as well as an action client that interacts with ROSPlan.

Action definition
------------------

Goal:
^^^^^^
* ``geometry_msgs/PoseStamped pose``: An end-effector goal pose

Result:
^^^^^^^^
* ``bool success``

Feedback:
^^^^^^^^^^
* ``string current_state``
* ``string message``

Launch file parameters:
^^^^^^^^^^^^^^^^^^^^^^^

The following parameters may be passed when launching the action server:  

* ``move_arm_server``: Name of the move_arm action server (default: 'move_arm_server')
* ``move_base_server``: Name of the move_base action server (default: 'move_base_server')
* ``gripper_controller_pkg_name``: The name of a package that implements functionalities for controlling a robot's gripper (default: 'mdr_gripper_controller')
* ``preplace_config_name``: Name of a default preplace configuration for sideways grasps (default: 'pregrasp')
* ``preplace_low_config_name``: Name of a preplace configuration for low sideways grasps (default: 'pregrasp_low')
* ``preplace_height_threshold``: Height threshold for choosing between low and high preplace configurations (default: 0.5)
* ``safe_arm_joint_config``: Name of a configuration in which the robot can safely move around the environment (default: 'folded')
* ``base_elbow_offset``: An optional offset between base_link and the manipulator's elbow; used for aligning the base with the placing pose so that the manipulator can easily reach it (default: -1)
* ``placing_dmp``: Path to a YAML file containing the weights of a dynamic motion primitive used for placing (default: '')
* ``dmp_tau``: The value of the temporal dynamic motion primitive parameter (default: 1)
* ``downward_placing_vel``: Velocity with which the arm moves downwards to detect impact with the placing surface (default: -0.02)
* ``placing_orientation``: For more constrained manipulators, it might make sense to use a fixed placing orientation (expressed as an (x, y, z, w) quaternion) to ensure easier reachability; for instance, we might want to keep the orientation with which an object was grasped instead of allowing arbitrary orientations (default: [], in which case the argument is ignored)

Action client:
---------------

The action client that interacts with ROSPlan inherits from the action client base class defined in mdr_rosplan_interface.

The following parameters need to be passed when launching the action client:

* ``action_name``: Name of the action as used in the planning domain
* ``server_name``: Name of the place action server
* ``action_timeout``: Maximum time (in seconds) that we are willing to wait for the action to be executed
* ``action_dispatch_topic``: Name of the topic at which the plan dispatcher sends action requests
* ``action_feedback_topic``: Name of the topic at which the action sends feedback to the plan dispatcher
* ``placing_pose_frame``: Name of the frame in which placing is performed (default: 'base_link')

Action execution summary:
-------------------------

The action performs placing with respect to the base_link frame (even if the goal pose is expressed in another frame) and is executed in a few steps:

* If base_elbow_offset is greater than 0, the base is aligned with the goal pose so that the origin of base_link is base_elbow_offset units away from it along the y-axis
* The manipulator is moved to a predefined manipulator configuration
* The end-effector is then sent to its placing goal and the gripper is opened; if a path to a dynamic motion primitive file is passed as a parameter to the action, the placing trajectory is represented by the motion primitive
* The manipulator is moved back to a configuration in which the robot can safely move around in the environment

Example usage:
--------------

1. Run the robot simulation: roslaunch ``mas_<robot>_bringup_sim robot.launch``
2. Run the moveit interface for the robot: ``roslaunch mas_<robot>_moveit move_group.launch``
3. Run the move_arm action server: ``roslaunch mas_<robot>_move_arm_action move_arm.launch``
4. Run the move_base action server: ``roslaunch mas_<robot>_move_base_action move_base.launch``
5. Run the action server: ``roslaunch mas_<robot>_place_action place_action.launch``
6. Run the client example: ``rosrun mdr_place_action place_action_client_test``; the client example uses a predefined placing pose
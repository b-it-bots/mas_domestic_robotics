Move Base Action
================

mdr_move_base_action
An action for moving a robot to a specified location.

The action includes an action server as well as an action client that interacts with ROSPlan.

Action definition
------------------

Goal types:
^^^^^^^^^^^^
* ``int32 NAMED_TARGET=0``
* ``int32 POSE=1``

Goal:
^^^^^^
* ``int32 goal_type``: The type of navigation goal (one of the allowed goal types defined above)
* ``string destination_location``: Navigation goal if ``goal_type`` is ``NAMED_TARGET``
* ``geometry_msgs/PoseStamped pose``: Navigation goal if ``goal_type`` is ``POSE``

Result:
^^^^^^^^
``bool success``

Feedback:
^^^^^^^^^^
*  ``string current_state``
*  ``string message``

Launch file parameters
------------------------

Action server
^^^^^^^^^^^^^^

The following parameters may be passed when launching the action server:

* ``move_base_server``: Name of the default ``move_base`` server (default: '/move_base')
* ``pose_description_file``: Name of a yaml file in which named goals are mapped to actual coordinates
* ``pose_frame``: Name of the frame in which the poses in ``pose_description_file`` are given (default: 'map')
* ``safe_arm_joint_config``: Name of a configuration specified in which the robot can safely move around the environment (default: 'folded')
* ``move_arm_server``: Name of the ``move_arm`` action server (default: 'move_arm_server')
* ``timeout``: Navigation timeout (in seconds) (default: 120)
* ``recovery_position_m_std``: If a navigation to a pose fails, an attempted recovery is to change the goal's (x, y) position a little and retry the navigation (the assumption is that the robot moves over the xy-plane)). The position is changed by adding Gaussian noise with zero mean and the given standard deviation (in meters) to the original goal position. (default: 0.2)
* ``max_recovery_attempts``: Maximum number of recovery attempts in case of navigation failures (default 1)

Action client
--------------

The action client that interacts with ROSPlan inherits from the action client base class defined in ``mdr_rosplan_interface``.

The following parameters need to be passed when launching the action client:

* ``action_name``: Name of the action as used in the planning domain
* ``server_name``: Name of the this action's server
* ``action_timeout``: Maximum time (in seconds) that we are willing to wait for the action to be executed
* ``action_dispatch_topic``: Name of the topic at which the plan dispatcher sends action requests
* ``action_feedback_topic``: Name of the topic at which the action sends feedback to the plan dispatcher

Action execution summary
-------------------------
This action is practically a wrapper around the default ``move_base`` functionality and thus sends navigation goals to the ``move_base`` server, with the addition that the robot's manipulator is sent to a safe configuration before motion commences.

Example usage:
^^^^^^^^^^^^^^
1. Run the robot simulation: ``roslaunch mas_<robot>_bringup_sim robot.launch``
2. Run the moveit interface for the robot: ``roslaunch mas_<robot>_moveit move_group.launch``
3. Run the move_arm action server: ``roslaunch mdr_move_arm_action move_arm.launch``
4. Run the action server: ``roslaunch mdr_move_base_action move_base_action.launch``
5. Run the client example: rosrun mdr_move_base_action ``move_base_action_client_test <NAMED_TARGET>``; the client example only allows using the action with a named target
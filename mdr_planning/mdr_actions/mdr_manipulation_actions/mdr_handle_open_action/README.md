# mdr_handle_open_action

An action for manipulating handles.

## Action definition

### Goal:

* ``string handle_type``: Type of the handle (currently unused)
* ``geometry_msgs/PoseStamped handle_pose``: Pose of the handle

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Launch file parameters

### Action server
* ``gripper_controller_pkg_name``: The name of a package that implements functionalities for controlling a robot's gripper (default: 'mdr_gripper_controller')
* ``move_arm_server``: Name of the move_arm action server (default: 'move_arm_server')
* ``move_base_server``: Name of the move_base action server (default: 'move_base_server')
* ``move_forward_server``: Name of the move_forward action server (default: 'move_forward_server')
* ``force_sensor_topic``: Name of topic for wrist force sensor measurements (default: 'force_sensor_topic')
* ``pregrasp_config_name``: Name of the pregrasp configuration (default: 'pregrasp_config_name')
* ``final_config_name``: Name of the final configuration (default: 'final_config_name')
* ``handle_open_dmp``: Path to a YAML file containing the weights of a dynamic motion primitive used for opening handles (default: '')
* ``dmp_tau``: The value of the temporal dynamic motion primitive parameter (default: 30)

## Dependencies

* ``numpy``
* ``scipy``
* ``cv2``
* ``cv_bridge``
* ``pyftsm``
* ``tf``
* ``actionlib``
* ``geometry_msgs``
* ``mas_execution``
* ``mdr_rosplan_interface``
* ``mdr_move_arm_action``
* ``mdr_move_base_action``
* ``mdr_move_forward_action``

## Learning data

The package additionally includes some data (under `learning_data`) that was collected in an experiment in which the robot was learning the best grasping position of a handle in our lab while standing at a predefined position in front of our handle. The robot was learning this by experience, namely by sampling grasping poses and labelling those based on the execution success; given those data, a success distribution was learned.

There are two sets of poses there:
* `sample_grasp_poses_uniform`: Poses sampled from a uniform distribution
* `sample_grasp_poses_learned`: Poses sampled from the learned success distribution

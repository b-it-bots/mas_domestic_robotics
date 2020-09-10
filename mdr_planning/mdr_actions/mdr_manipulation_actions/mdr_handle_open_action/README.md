# mdr_handle_open_action

An action for manipulating handles.

## Action definition

### Goal:

* ``string handle_type``: Optional type of the handle that needs to be opened

The following constants are also defined in the action goal:
* ``string PULL=pull_knob``
* ``string LEVER=lever_knob``
* ``string ROUND=round_knob``
* ``string UNKNOWN=unknown``

Note that these constants match the handle classes defined at https://github.com/b-it-bots/mas_models/blob/master/perception_models/detectors/classes_handles.yaml

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
* ``detect_handle_server``: Name of a detect_handle action server (default: 'detect_handle_server')
* ``force_sensor_topic``: Name of topic for wrist force sensor measurements (default: 'force_sensor_topic')
* ``pregrasp_config_name``: Name of the pregrasp configuration (default: 'pregrasp_config_name')
* ``final_config_name``: Name of the final configuration (default: 'final_config_name')
* ``handle_open_dmp``: Path to a YAML file containing the weights of a dynamic motion primitive used for opening handles (default: '')
* ``dmp_tau``: The value of the temporal dynamic motion primitive parameter (default: 30)

## Action execution summary

1. A handle is detected (including the type of handle and its pose)
2. Depending on the handle type, a different opening policy is followed:
    * a pull knob is the easiest to open since in that case the robot simply grasps the handle and moves backwards
    * a round knob is similar to a pull knob, but the robot moves sideways as well
    * for a lever knob, force feedback from the handle is used until the lever is pushed downwards and then the base moves to open the door

In all three cases, the pose at which the handle is grasped is sampled from a learned distribution as described below.

If `handle_type` is not `unknown`, the detected handle type is checked against the given type; if these don't match, an action failure is reported.

## Dependencies

* ``numpy``
* ``scipy``
* ``cv2``
* ``cv_bridge``
* ``pyftsm``
* ``tf``
* ``actionlib``
* ``geometry_msgs``
* ``mas_perception_msgs``
* ``mas_perception_libs``
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

## Handle detection

For the purpose of online handle detection, the action expects an action server of type `mas_perception_msgs/DetectObjectsAction` to be active. An example launch file for starting such a server is given at https://github.com/b-it-bots/mas_perception_libs/blob/devel/ros/launch/object_detection.launch. Note that a handle detection model can be found at https://github.com/b-it-bots/mas_models/tree/master/perception_models/detectors

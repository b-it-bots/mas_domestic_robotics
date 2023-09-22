Object Hand Over Action
=======================

mdr_hand_over_action
---------------------

An action for handing a grasped object over to a person. It is intended to be adaptive to different contexts of the task, such as the receiver's posture, the presence of an intermediate obstacle, receiver attributes, etc.

Currently, the action is only adaptive to postural context, through a hand-over position selection policy acquired using reinforcement learning.

The context-aware object hand-over demo scenario demonstrates the intended workflow of the action, including:

* Detecting a person
* Determining their posture
* Handing a grasped object over

Action definition
------------------

Goal:
^^^^^^
``string posture_type``: The receiver's posture: standing up, seated, or lying down
``bool obstacle``: Indicates whether an obstacle is present between the robot and the receiver
``bool release_detection``: Determines whether a force sensor-based object reception detection strategy is used
``bool context_aware``: Determines whether knowledge about context is utilized to adapt execution

Result:
^^^^^^^^
* ``bool success``

Feedback:
^^^^^^^^^
``string current_state``
``string message``

Launch file parameters
------------------------

Action server:
^^^^^^^^^^^^^^^

The following parameters may be passed when launching the action server:

* ``move_arm_server``: Name of the move_arm action server (default: 'move_arm_server')
* ``gripper_controller_pkg_name``: Name of a package that implements functionalities for controlling a robot's gripper (default: 'mdr_gripper_controller')
* ``init_config_name``: Name of the initial arm configuration for object hand-overs (default: 'neutral')
* ``hand_over_dmp``: Path to a YAML file containing the weights of a dynamic motion primitive used for object hand-overs (default: 'grasp.yaml')
* ``hand_over_position_policy_parameters_file``: Name of a YAML file containing the parameters of a learned context-dependent hand-over position policy (default: 'learned_position_policy_parameters.pkl')
* ``hand_over_policy_config_dir``: Path to a directory containing learned hand-over position policy parameters (default: '')
* ``hand_over_dmp_weights_dir``: Path to a directory containing learned hand-over trajectory shape policy parameters (default: '')
* ``dmp_tau``: The value of the temporal dynamic motion primitive parameter (default: 30)

Action execution summary
-------------------------

It is assumed that the robot starts with the object to be handed over already in its grasp. The action proceeds through the following steps:

1. The end-effector position at which the object will be handed over is first determined:

  -  if the ``context_aware`` goal parameter is set to ``True``, a position is sampled from learned policy, given the value of ``posture_type``
  -  otherwise, a pre-set, context-independent position is used.

2. The arm is moved to the ``neutral`` configuration.
3. The arm is then moved such that the end-effector ends in the position chosen in step 1 (currently the trajectory to be followed is set to a learned DMP trajectory).
4. If the ``release_detection`` action parameter is set to ``True``, the robot utilizes the wrist force sensor to determine when the person is pulling the object from its grasp with the intent of receiving it, using a signal change detection algorithm, and then releases the object. Otherwise, the robot waits for a short time and simply releases the object.
5. The arm is moved back to the ``neutral`` configuration.


Example usage:
^^^^^^^^^^^^^^^

* Run the ``move_arm`` action server: ``roslaunch mdr_move_arm_action move_arm.launch``
* Run the action server: ``roslaunch mdr_hand_over_action hand_over.launch``
* Run the client example: ``rosrun mdr_hand_over_action hand_over_action_client_test -p 'standing' ``. Additional arguments are: ``-c`` for context-aware execution, ``-r`` for release detection using the force-torque sensor and ``-o`` for execution with obstacle avoidance.
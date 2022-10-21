Receive Object Action
=====================

mdr_receive_object_action
--------------------------

An action for receiving an object from a person. The action is intended to be adaptive to different contexts of the task, such as the receiver's posture, receiver attributes, etc. The action is essentially equivalent to ``mdr_hand_over_action`` (the implementation of that action is reused almost completely), except that here the robot is waiting for an object to be placed in its hand.

Just as the hand-over action, the object reception action is currently only adaptive to postural context, through an object reception position selection policy acquired using reinforcement learning.

The context-aware object reception demo scenario demonstrates the intended workflow of the action, including:

* Detecting a person
* Determining their posture
* Receiving an object

Action definition
------------------

Goal:
^^^^^^

* ``string posture_type``: The receiver's posture: standing up, seated, or lying down
* ``bool obstacle``: Indicates whether an obstacle is present between the robot and the receiver
* ``bool reception_detection``: Determines whether a force sensor-based object reception detection strategy is used
* ``bool context_aware``: Determines whether knowledge about context is utilized to adapt execution

Result:
^^^^^^^^

* ``bool success``

Feedback:
^^^^^^^^^

* ``string current_state``
* ``string message``


Launch file parameters
-----------------------

Action server:
^^^^^^^^^^^^^^^

The following parameters may be passed when launching the action server:

* ``move_arm_server``: Name of the move_arm action server (default: 'move_arm_server')
* ``gripper_controller_pkg_name``: Name of a package that implements functionalities for controlling a robot's gripper (default: 'mdr_gripper_controller')
* ``init_config_name``: Name of the initial arm configuration for object reception (default: 'pregrasp')
* ``receive_object_dmp``: Path to a YAML file containing the weights of a dynamic motion primitive used for object reception (default: '')
* ``receive_object_position_policy_parameters_file``: Name of a YAML file containing the parameters of a learned context-dependent reception position policy (default: 'learned_position_policy_parameters.pkl')
* ``receive_object_policy_config_dir``: Path to a directory containing learned reception position policy parameters (default: '')
* ``dmp_tau``: The value of the temporal dynamic motion primitive parameter (default: 30)

Action execution summary
-------------------------

It is assumed that the robot starts without an object in its gripper and that the gripper is open. The action proceeds through the following steps:

1. The end-effector position at which the object will be received is first determined:

  *  if the ``context_aware`` goal parameter is set to ``True``, a position is sampled from learned policy, given the value of ``posture_type``,
  *  otherwise, a pre-set, context-independent position is used.

2. The arm is moved to the ``pregrasp`` configuration.
3. The arm is then moved such that the end-effector ends in the position chosen in step 1 (currently the trajectory to be followed is set to a learned DMP trajectory).
4. If the ``reception_detection`` action parameter is set to ``True``, the robot utilizes the wrist force sensor to determine when the person has put the object in the gripper with the intention of releasing it, using a signal change detection algorithm, and then grasps the object. Otherwise, the robot waits for a short time and simply closes the gripper.
5. The arm is moved back to the ``pregrasp`` configuration.

Example usage:
^^^^^^^^^^^^^^^

Run the ``move_arm`` action server: ``roslaunch mdr_move_arm_action move_arm.launch``
Run the action server: ``roslaunch mdr_receive_object_action receive_object.launch``
Run the client example: ``rosrun mdr_receive_object_action receive_object_action_client_test -p standing -r 1``, where ``p`` is a posture argument and ``r`` is 1 or 0 if the robot should detect when the object has been received
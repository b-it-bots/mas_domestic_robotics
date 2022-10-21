Enter Door Action
=================

mdr_enter_door_action
----------------------

An action for moving a robot through a door passage.

Action definition
------------------

Goal:
^^^^^
The action takes no inputs.

Result:
^^^^^^^^
* ``bool success``

Feedback:
^^^^^^^^^^
* ``string current_state``
* ``string message``


Example usage:
^^^^^^^^^^^^^^
* Run the action server and its dependencies (namely ``mdr_door_status`` and ``mdr_move_forward_action``): ``roslaunch mdr_enter_door_action enter_door.launch``
* Run the client example: ``rosrun mdr_enter_door_action enter_door_action_client_test``

Example usage (manual):
^^^^^^^^^^^^^^^^^^^^^^^^^

1. Launch the node that publishes the status of a door: ``roslaunch mdr_door_status door_status.launch``
2. Run the move forward action server: ``roslaunch mdr_move_forward_action move_forward.launch``
3. Run the action server: ``roslaunch mdr_enter_door_action enter_door_server.launch``
4. Run the client example: ``rosrun mdr_enter_door_action enter_door_action_client_test``

Simple action test (without using the door status node)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
1. Run the move forward action server: ``roslaunch mdr_move_forward_action move_forward.launch``
2. Run the action server: ``roslaunch mdr_enter_door_action enter_door_server.launch``
3. Run the client example: ``rosrun mdr_enter_door_action enter_door_action_client_test``
4. Publish an open door status: ``rostopic pub -r 1 /mdr_perception/door_status/door_status std_msgs/Bool "data: true"``
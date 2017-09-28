# mdr_move_forward_action

An action for moving a robot's base forward for a given number of seconds at a given speed.
This action does not take obstacles into account.

## Action definition

### Goal:

* ``float32 movement_duration``: The duration (in seconds) for which the base should move.
* ``float32 speed``: The speed with which we want to move the base.

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Dependencies

* ``geometry_msgs``

## Example usage

1. Run the action server: ``roslaunch mdr_move_forward_action move_forward.launch``
2. Run the client example: ``rosrun mdr_move_forward_action mdr_move_forward_action_client_test <duration> <speed>`` where ``<duration>`` is the duration (in seconds) for which the base should move and ``<speed>`` is the movement speed (in m/s)

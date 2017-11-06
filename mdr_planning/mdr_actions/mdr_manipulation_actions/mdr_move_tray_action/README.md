# mdr_move_tray_action

An action for moving Jenny's tray up or down.

## Action definition

### Goal:

* ``string direction``: Takes the values ``up`` or ``down``; these are defined as constants in the goal definition (``UP`` and ``DOWN`` respectively)

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Dependencies

* ``cob_script_server``

## Example usage

1. Run the cob script server: ``roslaunch cob_script_server script_server.launch``
2. Run the action server: ``roslaunch mdr_move_tray_action move_tray.launch``
3. Run the client example: ``rosrun mdr_move_tray_action move_tray_client_test <direction>`` where ``<direction>`` is either ``up`` or ``down``; the parameter ``<direction>`` is case-insensitive

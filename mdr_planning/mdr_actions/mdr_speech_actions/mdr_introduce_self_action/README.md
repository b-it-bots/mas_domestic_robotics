# mdr_introduce_self

An action for Jenny to introduce herself.

## Action definition

### Goal:

* ``profession (bool)``: Include what Jenny does for a living.
* ``residence (bool)``: Include where Jenny is from.
* ``date_of_birth (bool)``: Include the year Jenny arrived to the University

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Dependencies

* ``std_msgs``

## Example usage

2. Run the action server: ``roslaunch mdr_introduce_self_action introduce_self_action.launch``
3. Run the client example: ``rosrun mdr_introduce_self_action introduce_self_action_client_test ``

# mdr_detect_person

An action for detecting persons in a crowd

## Action definition

### Goal:

* ``bool start``: Takes the values ``True` or ``False`` indicating the start of the detection

### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string text``
* ``double[] faces``

## Dependencies


## Example usage

1. Run the action server: ``roslaunch mdr_detect_person detect_person.launch``
2. Run the client example: ``rosrun mdr_detect_person detect_person_client_test `` 
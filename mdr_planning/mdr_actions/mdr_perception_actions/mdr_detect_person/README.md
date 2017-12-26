# mdr_detect_person

An action for detecting persons in a crowd

## Action definition

### Goal:

* ``sensor_msgs/Image image``: Image in which faces should be detected
* ``bool start``: Takes the values ``True`` or ``False`` indicating the start of the detection

### Result:

* ``int64 number_of_faces``
* ``mdr_perception_msgs/FaceBoundingBox[] bounding_boxes``
* ``bool success``

### Feedback:

* ``string current_state``
* ``string text``

## Dependencies

* ``sensor_msgs``
* ``mdr_perception_msgs``
* ``numpy``
* ``cv2``
* ``cv_bridge``
* ``keras``

## Example usage

1. Run the action server: ``roslaunch mdr_detect_person detect_person.launch``
2. Run the client example: ``rosrun mdr_detect_person detect_person_client_test <input_image>``, where ``<input_image>`` is the path to a test image (e.g. the absolute path of ``mdr_detect_person/tests/data/detect_person_test_image.jpg``)

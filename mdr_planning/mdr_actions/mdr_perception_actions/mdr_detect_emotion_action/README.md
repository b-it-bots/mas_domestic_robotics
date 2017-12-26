# mdr_detect_emotion_action

An action for detecting emotions in a crowd.

## Action definition

### Goal:

* ``sensor_msgs/Image image``: Image in which emotions should be detected

### Result:

* ``int64 number_of_people``
* ``mdr_perception_msgs/FaceBoundingBox[] bounding_boxes``
* ``string[] emotions``
* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Dependencies

* ``sensor_msgs``
* ``mdr_perception_msgs``
* ``mdr_detect_person``
* ``numpy``
* ``cv2``
* ``cv_bridge``
* ``tensorflow``
* ``keras``

## Example usage

1. Run the detect person server: ``roslaunch mdr_detect_person detect_person.launch``
2. Run the action server: ``roslaunch mdr_detect_emotion_action detect_emotion.launch``
3. Run the client example: ``rosrun mdr_detect_emotion_action detect_emotion_action_client_test <input_image>``, where ``<input_image>`` is the path to a test image (e.g. the absolute path of ``mdr_detect_emotion_action/tests/data/detect_emotion_test_image.jpg``)

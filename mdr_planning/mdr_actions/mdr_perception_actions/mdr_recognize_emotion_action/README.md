# mdr_recognize_emotion_action

An action for recognising emotions in a crowd.

## Action definition

### Goal:

* ``sensor_msgs/Image image``: Image in which emotions should be recognised
* ``int64 number_of_faces``: Number of faces in the image
* ``mdr_perception_msgs/FaceBoundingBox[] bounding_boxes``: Bounding boxes of the faces in the image

### Result:

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
2. Run the action server: ``roslaunch mdr_recognize_emotion_action recognize_emotion.launch``
3. Run the client example: ``rosrun mdr_recognize_emotion_action recognize_emotion_action_client_test <input_image>``, where ``<input_image>`` is the path to a test image (e.g. the absolute path of ``mdr_recognize_emotion_action/tests/data/recognize_emotion_test_image.jpg``)

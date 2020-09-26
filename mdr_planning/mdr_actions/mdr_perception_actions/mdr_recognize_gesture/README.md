# mdr_recognize_gesture_action

An action for recognizing bodily gestures.

## Action definition

### Goal:

* ``string gesture_type``: Optional type of gesture; passing this in bypasses a gesture classification step


### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Launch file parameters

### Action server
* ``gesture_type``: The type of gesture (default: 'pointing')

## Action execution summary

...


## Dependencies

* ``numpy``
* ``openpose``
* ``cv2``
* ``pyftsm``
* ``actionlib``
* ``geometry_msgs``



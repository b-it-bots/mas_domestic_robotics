# mdr_recognize_gesture_action

*(Work In Progress)*

An action for recognizing bodily gestures, which involves two stages:
* Gesture classification: classifying an observed gesture acccording to a set of known gesture examples
* Gesture processing: reasoning about the classified gesture; for example, determining the object being pointed to after observing a pointing gesture.

The gesture classification procedure is inspired by the approach presented [in this paper](https://arxiv.org/abs/1906.12171). As a person performs an example of a gesture, the 3D positions of their joints (or keypoints) are captured from RGB camera frames using the `openpose` library. The resulting pose array data is then pre-processed and compared to a set of known (labeled) gesture examples, and classified according to the class of the most similar example (1NN classifier). The similarity of the observed gesture to all examples is determined using the Dynamic Time Warping (DTW) algorithm (provided by the [fastdtw](https://github.com/slaypni/fastdtw) library). Note that the input to the classifier can be either a camera stream or a video file.

The gesture processing step depends on the classified procedure. Currently, processing of only the pointing gesture is considered, and it involves inferring the pointing direction and using the output of an object classifier (*WIP*) to find the class of the object being pointed to. This relies on a simple projection of the line connecting the elbow and wrist joints, and determining the bounding box being intersected by the line (with simple heuristic rules for cases of multiple hypotheses).

### Scripts:
* ``save_gesture_video``: saves a video of a gesture of a given duration
* ``save_gesture_pose_data``: extracts and saves a pose array of a gesture from a camera or video file; pose arrays are of shape *(num_timesteps, num_keypoints, num_spatial_dims)*,  and are saved in `npy` format
* ``gesture_classifier_demo``: runs the gesture classifier on data from a camera or video file, and prints the inferred class
* ``pointing_gesture_demo``: demonstrates the inference of pointing direction by highlighting the bounding box of the object being pointed to got a camera stream.

## Action definition

### Goal:

* ``string gesture_type``: Optional type of gesture; passing this in bypasses the gesture classification step


### Result:

* ``bool success``

### Feedback:

* ``string current_state``
* ``string message``

## Launch file parameters

### Action server
* ``gesture_type``: The type of gesture (default: 'pointing')

## Action execution summary

TODO


## Dependencies

* ``numpy``
* ``openpose``
* ``cv2``
* ``fastdtw``
* ``pytorch``
* ``pyftsm``
* ``actionlib``
* ``geometry_msgs``
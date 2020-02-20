# ``mdr_perceive_plane_action``

An action for recognizing objects on a plane. The action includes an action server as well as an action client that
interacts with ROSPlan.

## Action definition

### Goal:

* ``string plane_config``: Name of the plane configuration to be perceived.
  TODO(minhnh): confirm that this can be removed.
* ``string plane_frame_prefix``: prefix to the plane names which will be written to the ``Plane`` objects in
  ``action_states.DetectObjects``

### Result:

* ``bool success``
* ``mas_perception_msgs/PlaneList recognized_planes``

### Feedback:

* ``string current_state``
* ``string message``

## Directory structure

```
├── CMakeLists.txt
├── package.xml
├── README.md
├── ros
│   ├── action
│   │   └── PerceivePlane.action
│   ├── launch
│   │   ├── perceive_plane_client.launch
│   │   ├── perceive_plane.launch
│   │   └── perceive_plane_test.launch
│   ├── scripts
│   │   ├── perceive_plane_action
│   │   ├── perceive_plane_client
│   │   └── perceive_plane_client_test
│   └── src
│       └── mdr_perceive_plane_action
│           ├── action_states.py
│           └── __init__.py
└── setup.py
```

## Launch file parameters

### Action server

The following arguments may be passed when launching the action server:
* ``target_frame``: name of the reference frame the object and plane poses will be transformed to
  (default: '/base_link')
* ``detection_action_name``: name of the action server used for object detection.
  Action file: `mas_perception_msgs/DetectScene.action`.
* ``recognition_service_name``: the name of the image recognition service for classifying object.
  Service file: `mas_perception_msgs/RecognizeImage.srv`.
* ``recognition_model_name``: the name of the image classification model located in the `mdr_object_recognition`
  package.
* ``preprocess_input_module``: the name of the module containing the image preprocessing function to be executed on
  images before inference.
* ``head_controller_pkg_name``: name of a package that implements functionalities
  for controlling a robot's head (default: 'mdr_head_controller')


### Action client

The action client that interacts with ROSPlan inherits from the action client base class defined in
``mdr_rosplan_interface``.

The following parameters need to be passed when launching the action client:
* ``action_name``: Name of the action as used in the planning domain
* ``server_name``: Name of the ``perceive_plane`` action server
* ``action_timeout``: Maximum time (in seconds) that we are willing to wait for the action to be executed
* ``clear_plane_memory``: If set to true, all objects previously seen on a plane will be deleted from the
  knowledge base before new objects are added to it (default false)
* ``action_dispatch_topic``: Name of the topic at which the plan dispatcher sends action requests
* ``action_feedback_topic``: Name of the topic at which the action sends feedback to the plan dispatcher

## Dependencies

* ``mas_perception_libs``
* ``mdr_object_recognition``
* ``mas_perception_msgs``

## Example usage

1. Change the ``service_module`` and ``service_class_name`` value to the desired extended service proxy. By default
   a test service proxy of type ``std_srv/Empty`` is loaded.
2. Run the action server: ``roslaunch mdr_perceive_plane_action perceive_plane.launch``.
   The ``perceive_plane_test.launch`` file will create a server of type ``std_srv/Empty`` for testing.
3. Run the client example: ``rosrun mdr_perceive_plane_action perceive_plane_client_test``

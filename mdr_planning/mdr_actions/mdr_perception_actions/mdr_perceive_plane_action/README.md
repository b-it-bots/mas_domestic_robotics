# ``mdr_perceive_plane_action``

An action for recognizing objects on a plane.
The action includes an action server as well as an action client that interacts with ROSPlan.

## Action definition

### Goal:

* ``string plane_config``: Name of the configuration set for setting up the dynamic parameter server.
 Each plane should have a set of configurations, and all configuration sets should be listed in
 ``config/perceive_plane_configurations.yaml``. This is necessary for configuring the scene segmentation
 node in ``mas_common_robotics``
* ``string plane_frame_prefix``: prefix to the plane names which will be written to the ``Plane`` objects in
 ``action_states.DetectObjects``

### Result:

* ``bool success``
* ``mcr_perception_msgs/PlaneList recognized_planes``

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
│   ├── config
│   │   └── perceive_plane_configurations.yaml
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
* ``service_module``: name of the module where an extension of ``mas_perception_libs.DetectionServiceProxy`` class is
defined, used for dynamically loading the extended class definition. See ``mas_perception_libs`` documentation for details
on how to use the extension class. (default: ``mdr_object_recognition``)
* ``service_class_name``: the name of the extended class (default: ``DetectionServiceProxyTest``)

### Action client

The action client that interacts with ROSPlan inherits from the action client base class defined in
``mdr_rosplan_interface``.

The following parameters need to be passed when launching the action client:
* ``action_name``: Name of the action as used in the planning domain
* ``server_name``: Name of the ``perceive_plane`` action server
* ``action_timeout``: Maximum time (in seconds) that we are willing to wait for the action to be executed
* ``action_dispatch_topic``: Name of the topic at which the plan dispatcher sends action requests
* ``action_feedback_topic``: Name of the topic at which the action sends feedback to the plan dispatcher
* ``knowledge_update_service``: Name of a service used for updating the planning problem as the world changes
* ``attribute_fetching_service``: Name of a service used for retrieving attributes representing the current knowledge
about the world

## Dependencies

* ``mdr_object_recognition_mean_circle``
* ``mas_perception_libs``
* ``mdr_object_recognition``
* ``mcr_dynamic_reconfigure_client``
* ``mcr_perception_msgs``

## Example usage

1. Change the ``service_module`` and ``service_class_name`` value to the desired extended service proxy. By default
a test service proxy of type ``std_srv/Empty`` is loaded.
2. Run the action server: ``roslaunch mdr_perceive_plane_action perceive_plane.launch``.
The ``perceive_plane_test.launch`` file will create a server of type ``std_srv/Empty`` for testing.
3. Run the client example: ``rosrun mdr_perceive_plane_action perceive_plane_client_test``

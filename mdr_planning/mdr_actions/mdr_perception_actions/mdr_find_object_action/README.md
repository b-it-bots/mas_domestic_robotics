# ``mdr_find_object_action``

An action for finding objects in a familiar environment. The action includes an action server as well as an action client that interacts with ROSPlan.

## Action definition

### Goal constants:

#### Goal types:

* ``int32 NAMED_OBJECT=1``
* ``int32 OBJECT_CATEGORY=2``

### Goal:

* ``int32 goal_type``: `NAMED_OBJECT` or `OBJECT_CATEGORY`
* ``string object_name``: Name of an object if `goal_type` is `NAMED_OBJECT`; name of an object category if `goal_type` is `OBJECT_CATEGORY`

### Result:

* ``bool success``: Indicates whether the action was successfully completed
* ``mas_perception_msgs/Object obj``: The object (if found)
* ``string object_location``: The location of the object if the object was found
* ``string relation``: The relation of the found object and its location (such as "on" or "in")

### Feedback:

* ``string current_state``
* ``string message``

## Directory structure

```
├── CMakeLists.txt
├── package.xml
├── README.md
├── setup.py
└── ros
    ├── action
    │   └── FindObject.action
    ├── launch
    │   ├── find_object.launch
    │   └── find_object_client.launch
    ├── scripts
    │   ├── find_object_action
    │   ├── find_object_client
    │   └── find_object_action_client_test
    └── src
        └── mdr_find_object_action
            ├── action_states.py
            └── __init__.py
```

## Launch file parameters

### Action server

The following arguments may be passed when launching the action server:

* ``ontology_url``: URL of an ontology file (if a local ontology is loaded, this parameter should be set to the the absolute path of the ontology prefixed by `file://`)
* ``ontology_class_prefix``: Prefix of the entities in the ontology
* ``retry_count_on_failure``: Number of times to retry the action if the execution fails (default 0)
* ``timeout_s``: Seconds to wait for the action to be executed (default 120.0)

### Action client

The action client that interacts with ROSPlan inherits from the action client base class defined in ``mdr_rosplan_interface``.

The following parameters need to be passed when launching the action client:
* ``action_name``: Name of the action as used in the planning domain
* ``server_name``: Name of the ``find_object`` action server
* ``action_timeout``: Maximum time (in seconds) that we are willing to wait for the action to be executed
* ``action_dispatch_topic``: Name of the topic at which the plan dispatcher sends action requests
* ``action_feedback_topic``: Name of the topic at which the action sends feedback to the plan dispatcher

## Dependencies

* ``mas_perception_libs``
* ``mas_perception_msgs``
* ``mas_knowledge_base``
* ``mdr_move_base_action``
* ``mdr_perceive_plane_action``

## Example usage

1. Run the robot simulation: ``roslaunch mas_<robot>_bringup_sim robot.launch``
2. Run the action server: ``roslaunch mas_<robot>_find_object_action find_object.launch``
3. Run the client example: ``rosrun mdr_find_object_action find_object_action_client_test``

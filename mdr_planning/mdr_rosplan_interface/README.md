# mdr_rosplan_interface

A package for interacting with the ROSPlan planning system.

## Existing Functionalities

### action_client_base

A base implementation of an action client class able to (i) respond to action dispatch messages sent by a ROSPlan dispatcher and (ii) update the knowledge base based on an executed action.

The base class defines the following methods:
* `call_action`: Callback method called when an action is dispatched. Only needs to listen to request for a particular action; all other requests should be ignored.
* `get_action_message`: Reads the parameters of a ROSPlan action request and generates an actionlib goal message for the action
* `update_knowledge_base`: Updates the knowledge base after an action has been executed
* `send_action_feedback`: Sends feedback to the plan dispatcher so that the dispatcher can continue with the plan execution

Action-specific implementations need to override `call_action`, `get_action_message`, and `update_knowledge_base`; `send_action_feedback` is already implemented in the base class. Example implementations of action clients can be found in the `mdr_actions` package (the [pickup action client](../mdr_actions/mdr_manipulation_actions/mdr_pickup_action/ros/scripts/pickup_client) is a representative example).

## Executable scripts

### init_planning_problem

A script for initialising the symbolic knowledge base with problem knowledge (object and predicate assertions) specified in a yaml file.

The initialisation file should have the following format:

```
objects:
    - obj:
        name: <string> | required
        type: <string> | required
    - obj:
        ...
init_state:
    - predicate:
        name: <string> | required
        args: | required
            - arg:
                key: <string> | required
                value: <string> | required
            - arg:
                ...
    - predicate:
        ...
```
where `objects` is a list of class assertions and `init_state` a list of predicate assertions. The specified knowledge can thus be seen as a prerequisite for generating a planning problem file.

## Launch file parameters

### action_client_base

The following parameters may be passed to the action client base:
* ``action_name``: Name of an action (e.g. `pick`); used for filtering action requests
* ``action_server_name``: Name of the actionlib server for executing the action
* ``action_timeout``: Maximum action duration; used as an actionlib timeout
* ``knowledge_update_service``: A remapped name of a service used for updating the symbolic knowledge base (in principle, this should be set to '/kcl_rosplan/update_knowledge_base')
* ``attribute_fetching_service``: A remapped name of a service used for retrieving knowledge from the symbolic knowledge base (in principle, this should be set to '/kcl_rosplan/get_current_knowledge')
* ``action_dispatch_topic``: A remapped name of a topic to which actions are dispatched by a ROSPlan plan dispatcher (in principle, this should be set to '/kcl_rosplan/action_dispatch')
* ``action_feedback_topic``: A remapped name of a topic at which a ROSPlan plan dispatcher listens for action feedback so that it can continue the plan execution (in principle, this should be set to '/kcl_rosplan/action_feedback')

### init_planning_problem

The following parameters should be passed when launching the `init_planning_problem` script:
* ``problem_file``: Absolute path to a file that contains initial problem knowledge (default '')
* ``kb_update_service``: (default '/kcl_rosplan/update_knowledge_base')

## Directory structure

```
mdr_rosplan_interface
|    CMakeLists.txt
|    package.xml
|    setup.py
|    README.md
|____ros
     |____launch
     |    |_____init_planning_problem.launch
     |    |
     |    scripts
     |    |_____init_planning_problem
     |    |
     |____src
          |____mdr_rosplan_interface
               |    __init__.py
               |____action_client_base.py
```
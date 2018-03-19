# ``mdr_execution_manager``

## Summary

Creates and executes a smach state machine from a state machine description specified in a configuration file.

State machine configuration files can be specified in two different ways:

* independently, i.e. a single configuration files fully describes a state machine
* one state machine can inherit the structure from another state machine, but can also add new states and remove/replace states that exist in the parent state machine

The latter allows defining generic/robot-independent state machines that can then be made specific by a robot-dependent definition. For example, the generic definition might contain an `arm_moving` state that might be defined somewhere in `mas_domestic_robotics`; however, this state might need to be reimplemented for a particular robot, such that the child configuration file allows us to specify the new state without redefining the complete state machine.

## Package organisation

The package has the following structure:
```
mdr_execution_manager
|    package.xml
|    CMakeLists.txt
|    setup.py
|    README.md
|____ros
     |____src
     |    |____mdr_execution_manager
     |    |    |    __init__.py
     |    |    |    sm_loader.py
     |    |    |____sm_params.py
     |    |
     |____scripts
          |____state_machine_creator

```

## State machine configuration file syntax

A state machine definition file is a `yaml` file with the following structure:
```
sm_id: <string> | required
states: list<string> | required
outcomes: list<string> | required
state_descriptions: | required
    - state:
        name: <string> | required
        state_module_name: <string> | required
        state_class_name: <string> | required
        transitions: | required
            - transition:
                name: <string> | required
                state: <string> | required
            - transition:
                ...
        arguments: | optional
            - argument:
                name: <string> | required
                value: <Python type> | required
            - argument:
                ...
    - state:
        ...
```

The configuration file keys have the following semantics:

* `sm_id`: a unique ID for the state machine
* `states`: a list of names for the state machine's states
* `outcomes`: a list of names of the state machine's terminal outcomes
* For each state, we have:
    * `name`: the state's name
    * `state_module_name`: the name of a module in which the state is defined
    * `state_class_name`: the name of the class that defines the state
    * For each transition, we have:
        * `name`: the transition's name
        * `state`: the name of the resulting state after the transition
    * If any parameters need to be passed to the state when it is being created, we can specify them here. For each of these, we have:
        * `name`: the argument's name
        * `value`: the argument's value

If a configuration file is supposed to inherit the structure from a parent configuration file, the configuration file has the following syntax:
```
sm_id: <string> | optional
states: list<string> | optional
outcomes: list<string> | optional
state_descriptions: | required
    - state:
        name: <string> | required
        remove: True
    - state:
        name: <string> | required
        state_module_name: <string> | required
        state_class_name: <string> | required
        transitions: | required
            - transition:
                name: <string> | required
                state: <string> | required
            - transition:
                ...
        arguments: | optional
            - argument:
                name: <string> | required
                value: <Python type> | required
            - argument:
                ...
```

The following rules apply for the child state machine configuration:

* `sm_id`: if not specified, the ID of the parent state machine will be used
* `states`: if not specified, the list of states specified in the parent's state machine will be used. States that are specified here, but are not in the parent's state machine will be added to the list of states
* `outcomes`: if not specified, the list of outcomes specified in the parent's state machine will be used. Outcomes that are specified here, but are not in the parent's state machine will be added to the list of outcomes
* For each state, if the `remove` key is defined, the state will be removed from the list of states (provided that it is already there). On the other hand, if a state has the same `name` as a state in the parent state machine configuration, but is completely redefined in the child configuration, the definition of the state from the child configuration file will be used


## State implementations

Loading a state machine through a configuration file imposes some constraints on the implementations of a state. In particular, a state cannot have named arguments unless they have default values; in addition, all arguments that are defined in the configuration file are passed to the state's constructor as `kwargs`.

A prototypical definition of a smach state that can be used together with a state machine definition file is shown below:
```
class StateName(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['outcome_1', ... 'outcome_n'],
                             output_keys=['key_1, ..., key_n'])
        self.arg = kwargs.get('arg_name', <default_value>)
        ...

    def execute(self, userdata):
        ...
```


## Examples

The example below shows a state machine definition for a simple scenario in which a robot needs to pick a bottle from a table.

```
sm_id: pick_bottle_from_table
states: [GO_TO_TABLE, FIND_OBJECT, GRASP_OBJECT]
outcomes: [DONE, FAILED]
state_descriptions:
    - state:
        name: GO_TO_TABLE
        state_module_name: mdr_navigation_states.states
        state_class_name: MoveBase
        transitions:
            - transition:
                name: succeeded
                state: FIND_OBJECT
            - transition:
                name: failed
                state: GO_TO_TABLE
            - transition:
                name: failed_after_retrying
                state: FAILED
        arguments:
            - argument:
                name: destination_locations
                value: [TABLE]
            - argument:
                name: number_of_retries
                value: 3
    - state:
        name: FIND_OBJECT
        state_module_name: mdr_perception_states.states
        state_class_name: LocateObject
        transitions:
            - transition:
                name: succeeded
                state: GRASP_OBJECT
            - transition:
                name: failed
                state: FIND_OBJECT
            - transition:
                name: failed_after_retrying
                state: FAILED
        arguments:
            - argument:
                name: object
                value: bottle_n
            - argument:
                name: number_of_retries
                value: 3
    - state:
        name: GRASP_OBJECT
        state_module_name: mdr_manipulation_states.states
        state_class_name: Pick
        transitions:
            - transition:
                name: succeeded
                state: DONE
            - transition:
                name: failed
                state: GRASP_OBJECT
            - transition:
                name: failed_after_retrying
                state: FAILED
        arguments:
            - argument:
                name: object
                value: bottle_n
            - argument:
                name: number_of_retries
                value: 3
```

This configuration file defines the following state machine:

![Pick bottle from table state machine](docs/figures/pick_bottle_from_table_sm.png)


The three states of the state machine will be implemented as smach states as shown below:
```
class MoveBase(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.destination_locations = kwargs.get('destination_locations', list())
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        ...

    def execute(self, userdata):
        ...
        success = <wait for the base to move and get the result>
        if success:
            return 'succeeded'
        else:
            if self.retry_count == self.number_of_retries:
                return 'failed_after_retrying'
            else:
                self.retry_count += 1
                return 'failed'


class LocateObject(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                             output_keys=['object_pose'])
        self.object = kwargs.get('object', '')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        ...

    def execute(self, userdata):
        ...
        success = <try to find the object and wait for the result>
        if success:
            return 'succeeded'
        else:
            if self.retry_count == self.number_of_retries:
                return 'failed_after_retrying'
            else:
                self.retry_count += 1
                return 'failed'


class Pick(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'])
        self.object = kwargs.get('object', '')
        self.number_of_retries = kwargs.get('number_of_retries', 0)
        self.retry_count = 0
        ...

    def execute(self, userdata):
        ...
        success = <try to grasp the object and wait for the result>
        if success:
            return 'succeeded'
        else:
            if self.retry_count == self.number_of_retries:
                return 'failed_after_retrying'
            else:
                self.retry_count += 1
                return 'failed'
```

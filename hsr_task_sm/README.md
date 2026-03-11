# HSR Task State Machines

**RoboCup@Home 2026 State Machine Framework**

A modular, configurable state machine framework for the Toyota HSR robot competing in RoboCup@Home challenges.

---

## Design Philosophy

**This package is designed to be SIMPLE and RELIABLE:**

| Feature | hsr_task_sm | mas_execution_manager |
|---------|-------------|----------------------|
| Knowledge Base | ❌ None | ✅ MongoDB + ROSPlan |
| Ontology | ❌ None | ✅ OWL files |
| Data Transfer | ✅ SMACH userdata | Mixed (KB + userdata) |
| Failure Mode | Graceful | KB crash = total failure |
| Complexity | Low | High |

### Key Principles:

1. **No Knowledge Base** - No MongoDB, no ROSPlan KB, no ontology
2. **Userdata Only** - All state data flows through SMACH userdata
3. **Direct Action Calls** - States call action servers directly
4. **Simple Recovery** - States handle their own retries
5. **Modular** - Each state is self-contained and testable

### Example Data Flow:
```
PERCEIVE_TABLE                  PICK_OBJECT
    │                               │
    └─→ userdata.detected_objects ──┘
                │
                └─→ object passed directly, no KB lookup
```

---

## Table of Contents

1. [Quick Start](#quick-start)
2. [Architecture Overview](#architecture-overview)
3. [Available Challenges](#available-challenges)
4. [State Reference](#state-reference)
5. [YAML Configuration](#yaml-configuration)
6. [Creating Custom States](#creating-custom-states)
7. [Creating Custom State Machines](#creating-custom-state-machines)
8. [Configuration Options](#configuration-options)
9. [Troubleshooting](#troubleshooting)

---

## Quick Start

### Run a challenge:
```bash
# Source the workspace
source ~/catkin_ws/devel/setup.bash

# Run a specific challenge
rosrun hsr_task_sm hri_challenge_sm
rosrun hsr_task_sm pick_place_challenge_sm
rosrun hsr_task_sm gpsr_challenge_sm
rosrun hsr_task_sm laundry_challenge_sm
rosrun hsr_task_sm restaurant_challenge_sm
rosrun hsr_task_sm finals_challenge_sm

# Or use YAML config (recommended)
rosrun hsr_task_sm yaml_sm_runner.py _config:=hri_challenge.yaml
```

### Visualize state machine:
```bash
rosrun smach_viewer smach_viewer.py
```

---

## Architecture Overview

```
┌─────────────────────────────────────────────────────────────────────┐
│                         YAML Configuration                          │
│                    (config/challenges/*.yaml)                        │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                      YAML State Machine Loader                       │
│                  (src/hsr_task_sm/yaml_sm_loader.py)                │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         SMACH State Machine                          │
│                                                                      │
│  ┌─────────┐    ┌─────────┐    ┌─────────┐    ┌─────────┐          │
│  │ State 1 │───▶│ State 2 │───▶│ State 3 │───▶│ State N │          │
│  └─────────┘    └─────────┘    └─────────┘    └─────────┘          │
└────────────────────────────┬────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        Reusable State Library                        │
│                                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │  Navigation  │  │  Perception  │  │ Manipulation │              │
│  │  NavigateTo  │  │ PerceiveTable│  │  PickObject  │              │
│  │  GoToGoal    │  │ DetectPerson │  │  PlaceObject │              │
│  │FollowPerson │  │              │  │   Handover   │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
│                                                                      │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐              │
│  │     HRI      │  │    Gaze      │  │  Furniture   │              │
│  │    Speak     │  │   LookAt     │  │   OpenDoor   │              │
│  │ListenCommand │  │ LookAtPerson │  │  CloseDoor   │              │
│  │              │  │  ResetGaze   │  │ OpenDrawer   │              │
│  └──────────────┘  └──────────────┘  └──────────────┘              │
└─────────────────────────────────────────────────────────────────────┘
                             │
                             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                         ROS Action Servers                           │
│  mdr_pickup_action, mdr_place_action, mdr_move_base_action, etc.    │
└─────────────────────────────────────────────────────────────────────┘
```

### Directory Structure

```
hsr_task_sm/
├── ros/
│   ├── config/
│   │   └── challenges/          # YAML state machine configs
│   │       ├── hri_challenge.yaml
│   │       ├── pick_place_challenge.yaml
│   │       ├── gpsr_challenge.yaml
│   │       └── ...
│   ├── scripts/                 # Executable state machines
│   │   ├── yaml_sm_runner.py    # YAML-based SM runner
│   │   ├── hri_challenge_sm
│   │   ├── pick_place_challenge_sm
│   │   └── ...
│   ├── src/hsr_task_sm/
│   │   ├── states/              # Reusable state implementations
│   │   │   ├── __init__.py
│   │   │   ├── speak.py
│   │   │   ├── navigate_to.py
│   │   │   └── ...
│   │   └── yaml_sm_loader.py    # YAML to SMACH converter
│   └── launch/
│       └── challenge.launch     # Generic launch file
└── README.md
```

---

## Available Challenges

| Challenge | Description | Script | YAML Config |
|-----------|-------------|--------|-------------|
| **HRI Challenge** | Receptionist task - greet guests, learn names/drinks | `hri_challenge_sm` | `hri_challenge.yaml` |
| **Pick & Place** | Clean table, serve breakfast | `pick_place_challenge_sm` | `pick_place_challenge.yaml` |
| **GPSR** | Execute 3 spoken commands | `gpsr_challenge_sm` | `gpsr_challenge.yaml` |
| **Laundry** | Fold clothes from hamper | `laundry_challenge_sm` | `laundry_challenge.yaml` |
| **Restaurant** | Serve customers in restaurant | `restaurant_challenge_sm` | `restaurant_challenge.yaml` |
| **Finals** | Household maintenance | `finals_challenge_sm` | `finals_challenge.yaml` |

---

## State Reference

### Navigation States

| State | Description | Parameters | Outcomes |
|-------|-------------|------------|----------|
| `NavigateTo` | Navigate to named location | `destination` | `succeeded`, `failed`, `failed_after_retrying` |
| `GoToGoal` | Navigate to pose | `goal_pose` | `succeeded`, `failed`, `failed_after_retrying` |
| `FollowPerson` | Follow detected person | `timeout` | `succeeded`, `lost_person`, `failed` |
| `ClearCostmap` | Clear navigation costmaps | - | `succeeded`, `failed` |

### Perception States

| State | Description | Parameters | Outcomes |
|-------|-------------|------------|----------|
| `PerceiveTable` | Perceive objects on surface | `plane_frame_prefix` | `succeeded`, `failed`, `failed_after_retrying` |
| `DetectPerson` | Detect person in view | `timeout` | `succeeded`, `no_person`, `failed` |
| `GetPersonFeatures` | Get person attributes | - | `succeeded`, `failed` |
| `RecognizePerson` | Recognize known person | - | `recognized`, `unknown`, `failed` |

### Manipulation States

| State | Description | Parameters | Outcomes |
|-------|-------------|------------|----------|
| `PickObject` | Pick up object | `picking_pose` | `succeeded`, `failed`, `failed_after_retrying` |
| `PlaceObject` | Place held object | `placing_pose` | `succeeded`, `failed`, `failed_after_retrying` |
| `HandoverToHuman` | Handover to person | `timeout` | `succeeded`, `failed`, `timeout` |
| `ReceiveFromHuman` | Receive from person | `timeout` | `succeeded`, `failed`, `timeout` |

### HRI States

| State | Description | Parameters | Outcomes |
|-------|-------------|------------|----------|
| `Speak` | Text-to-speech | `text` or `text_key` | `succeeded`, `failed` |
| `ListenForCommand` | Speech recognition | `timeout`, `grammar` | `succeeded`, `timeout`, `failed` |
| `ParseCommand` | Parse GPSR command | - | `succeeded`, `failed` |

### Gaze States

| State | Description | Parameters | Outcomes |
|-------|-------------|------------|----------|
| `LookAt` | Look at point | `target_point` | `succeeded`, `failed` |
| `LookAtPerson` | Look at detected person | - | `succeeded`, `failed` |
| `LookAtObject` | Look at object | `object_name` | `succeeded`, `failed` |
| `ResetGaze` | Reset head position | - | `succeeded`, `failed` |

### Furniture States

| State | Description | Parameters | Outcomes |
|-------|-------------|------------|----------|
| `OpenDoor` | Open door | `door_type` | `succeeded`, `failed`, `failed_after_retrying` |
| `CloseDoor` | Close door | `door_type` | `succeeded`, `failed`, `failed_after_retrying` |
| `OpenDrawer` | Open drawer | `drawer_name` | `succeeded`, `failed`, `failed_after_retrying` |
| `CloseDrawer` | Close drawer | `drawer_name` | `succeeded`, `failed`, `failed_after_retrying` |

---

## YAML Configuration

### Basic Structure

```yaml
# config/challenges/my_challenge.yaml

name: my_challenge
description: "Description of what this challenge does"

# Initial userdata (shared variables between states)
userdata:
  target_location: "kitchen"
  max_retries: 3

# State machine definition
states:
  - name: START
    type: Speak
    params:
      text: "Starting my challenge"
    transitions:
      succeeded: NAVIGATE
      failed: NAVIGATE

  - name: NAVIGATE
    type: NavigateTo
    params:
      destination_key: target_location  # Use userdata key
    transitions:
      succeeded: PERCEIVE
      failed: RETRY_NAV
      failed_after_retrying: DONE

  - name: PERCEIVE
    type: PerceiveTable
    transitions:
      succeeded: DONE
      failed: DONE

  - name: DONE
    type: Speak
    params:
      text: "Challenge complete"
    transitions:
      succeeded: SUCCEEDED  # Terminal state
      failed: SUCCEEDED

# Outcomes of the state machine (terminal states)
outcomes:
  - SUCCEEDED
  - FAILED
```

### Parameter Types

```yaml
# Static value
params:
  text: "Hello world"
  timeout: 10.0
  retries: 3

# From userdata (use _key suffix)
params:
  destination_key: target_location  # Gets value from userdata.target_location
  text_key: greeting_text           # Gets value from userdata.greeting_text

# From ROS param (use _param suffix)
params:
  timeout_param: ~perception_timeout  # Gets from ROS param server
```

### Loops and Conditionals

```yaml
states:
  # Loop back to same state
  - name: RETRY_PICK
    type: PickObject
    transitions:
      succeeded: PLACE
      failed: CHECK_RETRIES
  
  - name: CHECK_RETRIES
    type: CheckRetries
    params:
      max_retries: 3
    transitions:
      retry: RETRY_PICK
      max_reached: FAILED

  # Conditional based on userdata
  - name: CHECK_CONDITION
    type: CheckCondition
    params:
      condition_key: has_object
    transitions:
      'true': PLACE_OBJECT
      'false': PICK_OBJECT
```

### Sub-State Machines

```yaml
states:
  - name: PICK_AND_PLACE_SUB
    type: SubStateMachine
    params:
      config: pick_place_sub.yaml  # Load another YAML config
    transitions:
      SUCCEEDED: NEXT_STATE
      FAILED: HANDLE_ERROR
```

---

## Creating Custom States

### Step 1: Create State File

Create a new file in `ros/src/hsr_task_sm/states/`:

```python
# ros/src/hsr_task_sm/states/my_custom_state.py

import rospy
import smach

class MyCustomState(smach.State):
    """
    Description of what this state does.
    
    Parameters:
        my_param (str): Description of parameter
        timeout (float): Timeout in seconds
    
    Input Keys:
        input_data: Description of input
    
    Output Keys:
        output_data: Description of output
    
    Outcomes:
        succeeded: Task completed successfully
        failed: Task failed
        timeout: Task timed out
    """
    
    def __init__(self, my_param='default', timeout=10.0):
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'timeout'],
            input_keys=['input_data'],
            output_keys=['output_data']
        )
        self.my_param = my_param
        self.timeout = timeout
    
    def execute(self, userdata):
        rospy.loginfo('[MyCustomState] Starting with param: %s', self.my_param)
        
        # Access input data
        input_val = userdata.input_data
        
        # Do your work here
        try:
            result = self._do_work(input_val)
            userdata.output_data = result
            return 'succeeded'
        except Exception as e:
            rospy.logerr('[MyCustomState] Error: %s', e)
            return 'failed'
    
    def _do_work(self, data):
        # Implementation here
        return data
```

### Step 2: Register in `__init__.py`

```python
# ros/src/hsr_task_sm/states/__init__.py

from hsr_task_sm.states.my_custom_state import MyCustomState
```

### Step 3: Register in YAML Loader

```python
# ros/src/hsr_task_sm/yaml_sm_loader.py

STATE_REGISTRY = {
    # ... existing states ...
    'MyCustomState': MyCustomState,
}
```

### Step 4: Use in YAML

```yaml
states:
  - name: MY_STATE
    type: MyCustomState
    params:
      my_param: "custom_value"
      timeout: 15.0
    transitions:
      succeeded: NEXT
      failed: ERROR
      timeout: RETRY
```

---

## Creating Custom State Machines

### Option 1: YAML Configuration (Recommended)

1. Create a new YAML file in `ros/config/challenges/`:

```yaml
# ros/config/challenges/my_task.yaml
name: my_task
description: "My custom task"

userdata:
  location: "living_room"

states:
  - name: START
    type: Speak
    params:
      text: "Starting my task"
    transitions:
      succeeded: GO_TO_LOCATION
      failed: GO_TO_LOCATION

  - name: GO_TO_LOCATION
    type: NavigateTo
    params:
      destination_key: location
    transitions:
      succeeded: DONE
      failed: DONE
      failed_after_retrying: DONE

  - name: DONE
    type: Speak
    params:
      text: "Task complete"
    transitions:
      succeeded: SUCCEEDED
      failed: SUCCEEDED

outcomes:
  - SUCCEEDED
  - FAILED
```

2. Run it:

```bash
rosrun hsr_task_sm yaml_sm_runner.py _config:=$(rospack find hsr_task_sm)/config/challenges/my_task.yaml
```

### Option 2: Python Script

```python
#!/usr/bin/env python3
import rospy
import smach
import smach_ros
from hsr_task_sm.states import NavigateTo, Speak, PerceiveTable

def build_sm():
    sm = smach.StateMachine(outcomes=['SUCCEEDED', 'FAILED'])
    sm.userdata.location = 'kitchen'
    
    with sm:
        smach.StateMachine.add('START', 
            Speak(text='Starting'),
            transitions={'succeeded': 'NAVIGATE', 'failed': 'NAVIGATE'})
        
        smach.StateMachine.add('NAVIGATE',
            NavigateTo(destination='kitchen'),
            transitions={'succeeded': 'SUCCEEDED', 
                        'failed': 'FAILED',
                        'failed_after_retrying': 'FAILED'})
    
    return sm

if __name__ == '__main__':
    rospy.init_node('my_task_sm')
    sm = build_sm()
    outcome = sm.execute()
```

---

## Configuration Options

### ROS Parameters

Set via launch file or command line:

```xml
<!-- launch/challenge.launch -->
<launch>
    <arg name="challenge" default="hri_challenge"/>
    
    <node name="challenge_sm" pkg="hsr_task_sm" type="yaml_sm_runner.py" output="screen">
        <param name="config" value="$(find hsr_task_sm)/config/challenges/$(arg challenge).yaml"/>
        
        <!-- Global parameters -->
        <param name="speak_timeout" value="10.0"/>
        <param name="nav_timeout" value="60.0"/>
        <param name="perception_timeout" value="30.0"/>
    </node>
</launch>
```

### Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `HSR_TASK_DEBUG` | Enable debug logging | `false` |
| `HSR_TASK_SIM` | Simulation mode (skip real actions) | `false` |

---

## Troubleshooting

### Common Issues

**State machine not starting:**
```bash
# Check if action servers are running
rostopic list | grep -E "pickup|place|move_base"

# Check for errors
rosrun hsr_task_sm yaml_sm_runner.py _config:=my_config.yaml 2>&1 | tee log.txt
```

**Navigation failing:**
```bash
# Clear costmaps
rosservice call /move_base/clear_costmaps

# Check if goal is reachable
rostopic echo /move_base/status
```

**Perception not working:**
```bash
# Check camera topics
rostopic hz /camera/rgb/image_raw

# Visualize in rviz
rosrun rviz rviz -d $(rospack find hsr_task_sm)/config/perception.rviz
```

### Debug Mode

```bash
# Run with debug output
HSR_TASK_DEBUG=true rosrun hsr_task_sm yaml_sm_runner.py _config:=my_config.yaml

# Use SMACH viewer
rosrun smach_viewer smach_viewer.py
```

### Testing States Individually

```python
#!/usr/bin/env python3
import rospy
from hsr_task_sm.states import Speak

rospy.init_node('test_state')
state = Speak(text='Hello world')

class FakeUserdata:
    pass

ud = FakeUserdata()
outcome = state.execute(ud)
print(f'Outcome: {outcome}')
```

---

## Contributing

1. Create states in `ros/src/hsr_task_sm/states/`
2. Register in `__init__.py` and `yaml_sm_loader.py`
3. Add tests in `ros/test/`
4. Update this README

---

## License

BSD-3-Clause

## Authors

- RoboCup@Home Team 2026

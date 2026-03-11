# Quick Reference - State Machine YAML Config

## 🚀 Run a Challenge

```bash
# Using launch file (recommended)
roslaunch hsr_task_sm challenge.launch challenge:=hri_challenge

# Direct execution
rosrun hsr_task_sm yaml_sm_runner.py _config:=hri_challenge.yaml

# Custom config
rosrun hsr_task_sm yaml_sm_runner.py _config:=/path/to/my_config.yaml
```

## 🎨 Open GUI Editor

```bash
rosrun hsr_task_sm sm_editor_gui.py
rosrun hsr_task_sm sm_editor_gui.py /path/to/config.yaml
```

## 📋 YAML Structure

```yaml
name: my_challenge
description: "What this challenge does"

userdata:
  my_location: "kitchen"
  timeout: 10.0

states:
  - name: STATE_NAME
    type: StateType
    params:
      param1: "value"
      param2_key: userdata_variable  # Use _key suffix for userdata
    transitions:
      succeeded: NEXT_STATE
      failed: ERROR_STATE

outcomes:
  - SUCCEEDED
  - FAILED
```

## 🔧 Available State Types

### Navigation
| Type | Params | Outcomes |
|------|--------|----------|
| `NavigateTo` | `destination` | succeeded, failed, failed_after_retrying |
| `GoToGoal` | `goal_pose` | succeeded, failed, failed_after_retrying |
| `ClearCostmap` | - | succeeded, failed |
| `FollowPerson` | `timeout` | succeeded, lost_person, failed |

### Perception
| Type | Params | Outcomes |
|------|--------|----------|
| `PerceiveTable` | `plane_frame_prefix` | succeeded, failed, failed_after_retrying |
| `DetectPerson` | `timeout` | succeeded, no_person, failed |
| `RecognizePerson` | - | recognized, unknown, failed |

### Manipulation
| Type | Params | Outcomes |
|------|--------|----------|
| `PickObject` | - | succeeded, failed, failed_after_retrying |
| `PlaceObject` | - | succeeded, failed, failed_after_retrying |
| `HandoverToHuman` | `timeout` | succeeded, failed, timeout |

### HRI (Human-Robot Interaction)
| Type | Params | Outcomes |
|------|--------|----------|
| `Speak` | `text` or `text_key` | succeeded, failed |
| `ListenForCommand` | `timeout` | succeeded, timeout, failed |
| `ParseCommand` | - | succeeded, failed |

### Gaze
| Type | Params | Outcomes |
|------|--------|----------|
| `LookAt` | `target_point` | succeeded, failed |
| `LookAtPerson` | - | succeeded, failed |
| `ResetGaze` | - | succeeded, failed |

### Furniture
| Type | Params | Outcomes |
|------|--------|----------|
| `OpenDoor` | `door_type` | succeeded, failed, failed_after_retrying |
| `CloseDoor` | `door_type` | succeeded, failed, failed_after_retrying |

### Utility
| Type | Params | Outcomes |
|------|--------|----------|
| `Wait` | `duration` | succeeded |
| `CheckCondition` | `condition_key` | true, false |
| `CheckRetries` | `max_retries`, `counter_key` | retry, max_reached |
| `IncrementCounter` | `counter_key` | succeeded |
| `Log` | `message`, `level` | succeeded |

## 📝 Common Patterns

### Loop N Times
```yaml
states:
  - name: DO_WORK
    type: PickObject
    transitions:
      succeeded: INCREMENT
      failed: INCREMENT

  - name: INCREMENT
    type: IncrementCounter
    params:
      counter_key: loop_counter
    transitions:
      succeeded: CHECK_LOOP

  - name: CHECK_LOOP
    type: CheckRetries
    params:
      max_retries: 5
      counter_key: loop_counter
    transitions:
      retry: DO_WORK
      max_reached: DONE
```

### Retry on Failure
```yaml
states:
  - name: TRY_PICK
    type: PickObject
    transitions:
      succeeded: NEXT
      failed: RETRY_PICK
      failed_after_retrying: SKIP

  - name: RETRY_PICK
    type: Speak
    params:
      text: "Let me try again"
    transitions:
      succeeded: TRY_PICK
```

### Use Userdata Variable
```yaml
userdata:
  target: "kitchen"

states:
  - name: GO
    type: NavigateTo
    params:
      destination_key: target  # Uses userdata.target
```

## ✅ Validate Config

```bash
rosrun hsr_task_sm yaml_sm_runner.py _config:=my_config.yaml _validate_only:=true
```

## 👁️ Visualize State Machine

```bash
rosrun smach_viewer smach_viewer.py
```

## 📁 File Locations

```
hsr_task_sm/
├── ros/config/challenges/    # YAML configs
│   ├── hri_challenge.yaml
│   ├── pick_place_challenge.yaml
│   ├── gpsr_challenge.yaml
│   ├── laundry_challenge.yaml
│   ├── restaurant_challenge.yaml
│   └── finals_challenge.yaml
├── ros/scripts/
│   ├── yaml_sm_runner.py    # Run YAML configs
│   └── sm_editor_gui.py     # GUI editor
└── ros/src/hsr_task_sm/
    ├── states/              # State implementations
    └── yaml_sm_loader.py    # YAML parser
```

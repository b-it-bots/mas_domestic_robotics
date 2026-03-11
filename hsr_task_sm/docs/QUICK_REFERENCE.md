# Quick Reference Card

Print this and keep it next to you! 📄

---

## State Machine = Recipe for Robot

```
[Step 1] --success--> [Step 2] --success--> [Step 3] --> DONE!
   |                     |
   v                     v
 failed                failed
   |                     |
   v                     v
 retry!               retry!
```

---

## YAML Template (Copy This!)

```yaml
name: my_task
description: "What this task does"

userdata:
  my_variable: "value"

states:
  - name: START
    type: ClearCostmap
    transitions:
      succeeded: FIRST_REAL_STATE
      failed: FIRST_REAL_STATE

  # YOUR STATES HERE

  - name: SUCCEEDED
    type: Wait
    params:
      duration: 0.1
    transitions:
      succeeded: null

  - name: FAILED
    type: Wait
    params:
      duration: 0.1
    transitions:
      succeeded: null
```

---

## State Types Cheat Sheet

| I want to... | Use this type | Key param |
|--------------|---------------|-----------|
| Go somewhere | `NavigateTo` | `destination: "kitchen"` |
| Say something | `Speak` | `text: "Hello"` |
| Listen to speech | `ListenForCommand` | `timeout: 10.0` |
| Find a person | `DetectPerson` | `timeout: 30.0` |
| See objects | `PerceiveTable` | - |
| Pick up object | `PickObject` | `object_name: "cup"` |
| Put down object | `PlaceObject` | - |
| Give to person | `HandoverToHuman` | - |
| Clear nav memory | `ClearCostmap` | - |
| Wait/Pause | `Wait` | `duration: 3.0` |

---

## Transitions (What Happens Next)

```yaml
transitions:
  succeeded: NEXT_STATE           # ✅ It worked! Go here
  failed: SAME_STATE              # ❌ Retry (loop back)
  failed_after_retrying: FAILED   # 💀 Give up
```

**Special for some states:**
- `no_person_found` - DetectPerson didn't see anyone
- `timeout` - Ran out of time

---

## Retry Pattern (Use This!)

```yaml
- name: DO_THING
  type: SomeType
  params:
    retries: 3                    # Try 3 times
  transitions:
    succeeded: NEXT
    failed: DO_THING              # Loop back = retry
    failed_after_retrying: FAILED # Gave up
```

---

## Hardcoded vs Dynamic Values

| Style | Example | Meaning |
|-------|---------|---------|
| `destination: "kitchen"` | Hardcoded | Always kitchen |
| `destination_key: target` | From userdata | Whatever `target` says |

---

## Save Data from State

```yaml
- name: LISTEN
  type: ListenForCommand
  params:
    timeout: 10.0
  transitions:
    succeeded: USE_IT
    failed: LISTEN
    failed_after_retrying: FAILED
  remapping:
    recognized_command: what_they_said   # Save here!
```

Later use it:
```yaml
- name: GO_THERE
  type: NavigateTo
  params:
    destination_key: what_they_said      # Use it!
```

---

## Locations You Can Use

```
home            door1_inside      door1_outside
kitchen         bedroom           living_room
dining_table    living_room_table bedside_table
fridge          dish_washer       cabinet
shelf           washing_machine
```

---

## Run Your Task

```bash
roslaunch hsr_task_sm task_sm.launch config:=my_task.yaml
```

---

## Debug / Test

```bash
# Test individual states
rosrun hsr_task_sm state_tester_cli.py

# See ROS logs
rostopic echo /rosout
```

---

## Common Mistakes

❌ `type: navigate_to` → ✅ `type: NavigateTo` (CamelCase!)

❌ `transitions: succeeded: NEXT` → ✅ Use proper indentation!

❌ Forgot `SUCCEEDED` and `FAILED` end states

❌ `failed: NEXT` → You probably want `failed: SAME_STATE` to retry

---

## Still Stuck?

1. Copy `template_robust.yaml` and modify it
2. Check existing examples in `config/challenges/`
3. Ask for help!

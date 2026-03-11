# HSR Task State Machine - Beginner's Guide

Welcome! This guide will teach you how to make your robot do cool stuff using **state machines**. Don't worry if that sounds complicated - by the end of this guide, you'll be creating your own robot tasks!

---

## What is a State Machine?

Think of a state machine like a **recipe** or a **flowchart** for your robot.

Imagine making a sandwich:
1. **Get bread** → if success, go to step 2
2. **Add cheese** → if success, go to step 3
3. **Close sandwich** → Done!

Each step is a **state**. The robot does one thing at a time, then moves to the next step based on what happened.

```
[Get Bread] --success--> [Add Cheese] --success--> [Close Sandwich] --success--> DONE!
     |                        |
     v                        v
   failed                   failed
     |                        |
     v                        v
   FAILED                   FAILED
```

---

## How This Works

Instead of writing Python code for every task, you write a simple **YAML file** (like a config file). The system reads it and builds the state machine for you!

**Your YAML file** → **System builds state machine** → **Robot executes it**

---

## Your First State Machine

Let's make a simple task: the robot says hello and goes to the kitchen.

### Step 1: Create the File

Create a new file called `my_first_task.yaml` in:
```
hsr_task_sm/ros/config/challenges/my_first_task.yaml
```

### Step 2: Write the YAML

```yaml
# My First Robot Task
# This robot says hello and goes to the kitchen!

name: my_first_task
description: "Say hello and go to kitchen"

# Variables (data the robot remembers)
userdata:
  kitchen_location: "kitchen"

# The steps (states)
states:
  # Step 1: Clear navigation memory
  - name: START
    type: ClearCostmap
    transitions:
      succeeded: SAY_HELLO
      failed: SAY_HELLO

  # Step 2: Say hello
  - name: SAY_HELLO
    type: Speak
    params:
      text: "Hello! I am going to the kitchen now."
      retries: 2
    transitions:
      succeeded: GO_TO_KITCHEN
      failed: SAY_HELLO
      failed_after_retrying: GO_TO_KITCHEN

  # Step 3: Navigate to kitchen
  - name: GO_TO_KITCHEN
    type: NavigateTo
    params:
      destination: "kitchen"
      retries: 3
    transitions:
      succeeded: SAY_ARRIVED
      failed: GO_TO_KITCHEN
      failed_after_retrying: FAILED

  # Step 4: Announce arrival
  - name: SAY_ARRIVED
    type: Speak
    params:
      text: "I have arrived at the kitchen!"
      retries: 2
    transitions:
      succeeded: SUCCEEDED
      failed: SAY_ARRIVED
      failed_after_retrying: SUCCEEDED

  # End states (every state machine needs these!)
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

### Step 3: Run It

On the robot:
```bash
roslaunch hsr_task_sm task_sm.launch config:=my_first_task.yaml
```

---

## Understanding the YAML Structure

Let's break down each part:

### 1. Header
```yaml
name: my_first_task           # Name of your task (no spaces!)
description: "What it does"   # Description for humans
```

### 2. Userdata (Robot's Memory)
```yaml
userdata:
  my_variable: "some value"
  another_one: 42
```

This is like giving the robot a notepad to remember things.

### 3. States (The Steps)
```yaml
states:
  - name: STATE_NAME          # Unique name (use CAPS_WITH_UNDERSCORES)
    type: StateType           # What kind of action (see list below)
    params:                   # Settings for this action
      param1: value1
      param2: value2
    transitions:              # What to do next
      succeeded: NEXT_STATE   # If it works
      failed: RETRY_STATE     # If it fails
```

---

## Available State Types

Here are all the things your robot can do:

### Navigation (Moving Around)

| Type | What it does | Example |
|------|--------------|---------|
| `NavigateTo` | Go to a named location | `destination: "kitchen"` |
| `ClearCostmap` | Clear navigation memory | (no params needed) |

```yaml
- name: GO_TO_KITCHEN
  type: NavigateTo
  params:
    destination: "kitchen"    # Where to go
    retries: 3                # Try 3 times if it fails
  transitions:
    succeeded: NEXT_STATE
    failed: GO_TO_KITCHEN     # Retry
    failed_after_retrying: FAILED
```

**Available locations:**
- `home`, `kitchen`, `bedroom`, `living_room`
- `door1_inside`, `door1_outside`
- `dining_table`, `living_room_table`
- `fridge`, `dish_washer`, `cabinet`, `shelf`

### Speaking

| Type | What it does | Example |
|------|--------------|---------|
| `Speak` | Say something | `text: "Hello!"` |

```yaml
- name: SAY_HELLO
  type: Speak
  params:
    text: "Hello, I am your robot!"
    retries: 2
  transitions:
    succeeded: NEXT_STATE
    failed: SAY_HELLO
    failed_after_retrying: NEXT_STATE  # Continue anyway
```

### Listening

| Type | What it does | Output |
|------|--------------|--------|
| `ListenForCommand` | Listen for speech | `recognized_command` |

```yaml
- name: LISTEN
  type: ListenForCommand
  params:
    timeout: 10.0             # Wait 10 seconds
    retries: 3
  transitions:
    succeeded: USE_COMMAND
    failed: LISTEN
    failed_after_retrying: ASK_AGAIN
  remapping:
    recognized_command: what_user_said  # Save to userdata
```

### Perception (Seeing)

| Type | What it does | Output |
|------|--------------|--------|
| `DetectPerson` | Find a person | `person_pose` |
| `PerceiveTable` | See objects on table | `detected_objects` |

```yaml
- name: FIND_PERSON
  type: DetectPerson
  params:
    timeout: 30.0
    retries: 2
  transitions:
    succeeded: GREET_PERSON
    no_person_found: WAIT_FOR_PERSON
    failed_after_retrying: FAILED
```

### Manipulation (Grabbing Stuff)

| Type | What it does |
|------|--------------|
| `PickObject` | Pick up an object |
| `PlaceObject` | Put down an object |
| `HandoverToHuman` | Give object to person |
| `ReceiveFromHuman` | Take object from person |

```yaml
- name: PICK_CUP
  type: PickObject
  params:
    object_name: "cup"
  transitions:
    succeeded: DELIVER
    failed: FAILED
```

### Utility (Helpers)

| Type | What it does |
|------|--------------|
| `Wait` | Pause for X seconds |
| `Log` | Print a debug message |

```yaml
- name: WAIT_A_BIT
  type: Wait
  params:
    duration: 3.0             # Wait 3 seconds
  transitions:
    succeeded: NEXT_STATE
```

---

## Retry Logic (Handling Failures)

Every state has **built-in retry logic**. Here's how it works:

```yaml
- name: GO_SOMEWHERE
  type: NavigateTo
  params:
    destination: "kitchen"
    retries: 3                # Try up to 3 times
  transitions:
    succeeded: NEXT           # Worked! Move on
    failed: GO_SOMEWHERE      # Failed, try again (loops back)
    failed_after_retrying: FAILED  # Gave up after 3 tries
```

The flow looks like this:
```
[GO_SOMEWHERE] --succeeded--> [NEXT]
      |
      | failed (retry 1)
      v
[GO_SOMEWHERE] --succeeded--> [NEXT]
      |
      | failed (retry 2)
      v
[GO_SOMEWHERE] --succeeded--> [NEXT]
      |
      | failed (retry 3 - max reached!)
      v
[failed_after_retrying] --> [FAILED]
```

---

## Passing Data Between States

Sometimes you want one state to give information to another. 

### Example: Ask user where to go, then go there

```yaml
userdata:
  target_location: ""         # Empty, will be filled by LISTEN state

states:
  - name: ASK_WHERE
    type: Speak
    params:
      text: "Where should I go?"
      retries: 2
    transitions:
      succeeded: LISTEN_LOCATION
      failed: ASK_WHERE
      failed_after_retrying: LISTEN_LOCATION

  - name: LISTEN_LOCATION
    type: ListenForCommand
    params:
      timeout: 10.0
      retries: 3
    transitions:
      succeeded: GO_THERE
      failed: LISTEN_LOCATION
      failed_after_retrying: ASK_WHERE
    remapping:
      recognized_command: target_location  # SAVE what user said

  - name: GO_THERE
    type: NavigateTo
    params:
      destination_key: target_location     # USE what user said
      retries: 3
    transitions:
      succeeded: DONE
      failed: GO_THERE
      failed_after_retrying: FAILED
```

### The Magic Words

| Param Style | Meaning | Example |
|-------------|---------|---------|
| `destination: "kitchen"` | Hardcoded value | Always goes to kitchen |
| `destination_key: target_location` | Read from userdata | Goes wherever `target_location` says |
| `text: "Hello"` | Hardcoded text | Always says "Hello" |
| `text_key: message` | Read from userdata | Says whatever `message` contains |

### Remapping (Renaming Data)

When a state outputs data, you can rename it:

```yaml
remapping:
  state_output_name: your_userdata_name
```

Example:
```yaml
- name: LISTEN
  type: ListenForCommand
  ...
  remapping:
    recognized_command: guest_name    # Save as "guest_name" in userdata
```

---

## Creating Your Own State (Python)

Want to do something the built-in states can't do? Make your own!

### Step 1: Create the Python File

Create `hsr_task_sm/ros/src/hsr_task_sm/states/my_custom_state.py`:

```python
#!/usr/bin/env python3
"""My custom state that does cool stuff."""

import rospy
import smach


class MyCustomState(smach.State):
    """
    Description of what this state does.
    
    Params:
        my_param: What this parameter controls
        retries: Number of retry attempts
    
    Outcomes:
        succeeded: It worked!
        failed: Something went wrong, will retry
        failed_after_retrying: Gave up after max retries
    
    Output:
        result_data: The result we computed
    """

    def __init__(self, my_param="default", retries=2):
        # Declare what we read and write
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'failed', 'failed_after_retrying'],
            input_keys=[],                    # What we READ from userdata
            output_keys=['result_data']       # What we WRITE to userdata
        )
        
        # Save parameters
        self.my_param = my_param
        self.retries = retries
        self.retry_count = 0

    def execute(self, userdata):
        """This runs when the state executes."""
        rospy.loginfo('[MyCustomState] Running with param: %s', self.my_param)
        
        try:
            # ========== YOUR CODE HERE ==========
            # Do whatever you want!
            result = self.do_something_cool()
            
            # Write result to userdata
            userdata.result_data = result
            
            # Success!
            self.retry_count = 0
            return 'succeeded'
            
        except Exception as e:
            rospy.logerr('[MyCustomState] Error: %s', str(e))
            return self._retry()

    def do_something_cool(self):
        """Your custom logic goes here."""
        return "Hello from my custom state!"

    def _retry(self):
        """Handle retry logic (copy this exactly)."""
        if self.retry_count >= self.retries:
            self.retry_count = 0
            rospy.logerr('[MyCustomState] Max retries reached')
            return 'failed_after_retrying'
        self.retry_count += 1
        rospy.logwarn('[MyCustomState] Retry %d/%d', self.retry_count, self.retries)
        return 'failed'
```

### Step 2: Register Your State

Edit `hsr_task_sm/ros/src/hsr_task_sm/states/__init__.py`:

```python
from .my_custom_state import MyCustomState
```

Edit `hsr_task_sm/ros/src/hsr_task_sm/yaml_sm_loader.py`:

```python
from hsr_task_sm.states import MyCustomState

STATE_REGISTRY = {
    # ... existing states ...
    'MyCustomState': MyCustomState,  # Add this line!
}
```

### Step 3: Use It in YAML

```yaml
- name: DO_COOL_THING
  type: MyCustomState
  params:
    my_param: "awesome"
    retries: 3
  transitions:
    succeeded: NEXT_STATE
    failed: DO_COOL_THING
    failed_after_retrying: FAILED
  remapping:
    result_data: cool_result      # Save output to userdata
```

---

## Complete Example: Fetch Task

Here's a complete task where the robot:
1. Asks what object to fetch
2. Goes to the kitchen
3. Picks up the object
4. Brings it back

```yaml
name: fetch_object
description: "Ask user what to fetch, go get it, bring it back"

userdata:
  object_to_fetch: ""
  pickup_location: "kitchen"
  delivery_location: "living_room_table"

states:
  # ===== START =====
  - name: START
    type: ClearCostmap
    transitions:
      succeeded: GREET
      failed: GREET

  # ===== ASK USER =====
  - name: GREET
    type: Speak
    params:
      text: "Hello! What object should I fetch for you?"
      retries: 2
    transitions:
      succeeded: LISTEN_OBJECT
      failed: GREET
      failed_after_retrying: LISTEN_OBJECT

  - name: LISTEN_OBJECT
    type: ListenForCommand
    params:
      timeout: 15.0
      retries: 3
    transitions:
      succeeded: CONFIRM_OBJECT
      failed: LISTEN_OBJECT
      failed_after_retrying: GREET
    remapping:
      recognized_command: object_to_fetch

  - name: CONFIRM_OBJECT
    type: Speak
    params:
      text: "Got it! I will fetch that for you. Please wait here."
      retries: 2
    transitions:
      succeeded: GO_TO_KITCHEN
      failed: CONFIRM_OBJECT
      failed_after_retrying: GO_TO_KITCHEN

  # ===== FETCH OBJECT =====
  - name: GO_TO_KITCHEN
    type: NavigateTo
    params:
      destination_key: pickup_location
      retries: 3
    transitions:
      succeeded: LOOK_FOR_OBJECT
      failed: CLEAR_AND_RETRY_NAV
      failed_after_retrying: NAV_FAILED

  - name: CLEAR_AND_RETRY_NAV
    type: ClearCostmap
    transitions:
      succeeded: GO_TO_KITCHEN
      failed: GO_TO_KITCHEN

  - name: LOOK_FOR_OBJECT
    type: PerceiveTable
    params:
      retries: 3
    transitions:
      succeeded: PICK_OBJECT
      failed: LOOK_FOR_OBJECT
      failed_after_retrying: PERCEPTION_FAILED

  - name: PICK_OBJECT
    type: PickObject
    params:
      object_name_key: object_to_fetch
    transitions:
      succeeded: RETURN_TO_USER
      failed: PICK_FAILED

  # ===== DELIVER =====
  - name: RETURN_TO_USER
    type: NavigateTo
    params:
      destination_key: delivery_location
      retries: 3
    transitions:
      succeeded: HANDOVER
      failed: RETURN_TO_USER
      failed_after_retrying: NAV_FAILED

  - name: HANDOVER
    type: HandoverToHuman
    transitions:
      succeeded: ANNOUNCE_DONE
      failed: ANNOUNCE_DONE

  # ===== SUCCESS =====
  - name: ANNOUNCE_DONE
    type: Speak
    params:
      text: "Here you go! Task completed."
      retries: 2
    transitions:
      succeeded: SUCCEEDED
      failed: ANNOUNCE_DONE
      failed_after_retrying: SUCCEEDED

  - name: SUCCEEDED
    type: Wait
    params:
      duration: 0.1
    transitions:
      succeeded: null

  # ===== FAILURE HANDLERS =====
  - name: NAV_FAILED
    type: Speak
    params:
      text: "Sorry, I could not navigate to the destination."
      retries: 2
    transitions:
      succeeded: FAILED
      failed: FAILED
      failed_after_retrying: FAILED

  - name: PERCEPTION_FAILED
    type: Speak
    params:
      text: "Sorry, I could not find the object."
      retries: 2
    transitions:
      succeeded: FAILED
      failed: FAILED
      failed_after_retrying: FAILED

  - name: PICK_FAILED
    type: Speak
    params:
      text: "Sorry, I could not pick up the object."
      retries: 2
    transitions:
      succeeded: FAILED
      failed: FAILED
      failed_after_retrying: FAILED

  - name: FAILED
    type: Wait
    params:
      duration: 0.1
    transitions:
      succeeded: null
```

---

## Tips and Tricks

### 1. Always have SUCCEEDED and FAILED end states
```yaml
- name: SUCCEEDED
  type: Wait
  params:
    duration: 0.1
  transitions:
    succeeded: null    # null means "end here"

- name: FAILED
  type: Wait
  params:
    duration: 0.1
  transitions:
    succeeded: null
```

### 2. Use ClearCostmap before navigation
If navigation keeps failing, clear the costmap first:
```yaml
- name: CLEAR_NAV
  type: ClearCostmap
  transitions:
    succeeded: TRY_NAV_AGAIN
    failed: TRY_NAV_AGAIN
```

### 3. Speech is not critical - continue on failure
```yaml
- name: SAY_SOMETHING
  type: Speak
  params:
    text: "Hello"
    retries: 2
  transitions:
    succeeded: NEXT
    failed: SAY_SOMETHING
    failed_after_retrying: NEXT    # Continue anyway!
```

### 4. Name your states clearly
Use descriptive names:
- Good: `GO_TO_KITCHEN`, `ASK_FOR_NAME`, `PICK_UP_CUP`
- Bad: `STATE1`, `DO_THING`, `NEXT`

### 5. Test one state at a time
Use the state tester to debug:
```bash
rosrun hsr_task_sm state_tester_cli.py
```

---

## Troubleshooting

### "Unknown state type: X"
You misspelled the state type. Check the available types list above.

### "State X has no transition for outcome Y"
You forgot to handle all possible outcomes. Check what outcomes your state type has.

### Robot keeps retrying forever
You set `failed: SAME_STATE` but the state keeps failing. Check:
1. Is the service/action server running?
2. Is the parameter correct?
3. Add `failed_after_retrying: FAILED` to give up eventually.

### Navigation always fails
1. Check if the location name exists in `navigation_goals.yaml`
2. Try adding `ClearCostmap` before navigation
3. Make sure the robot can physically reach that location

---

## Quick Reference Card

```yaml
# Basic state structure
- name: STATE_NAME
  type: StateType
  params:
    param1: value1
    retries: 3
  transitions:
    succeeded: NEXT_STATE
    failed: STATE_NAME           # Retry
    failed_after_retrying: FAILED
  remapping:
    state_output: userdata_key

# Common transitions
transitions:
  succeeded: NEXT      # It worked
  failed: SAME         # Try again
  failed_after_retrying: FAILED  # Give up

# Hardcoded vs dynamic values
destination: "kitchen"           # Always kitchen
destination_key: my_location     # Use userdata.my_location

text: "Hello"                    # Always say "Hello"
text_key: my_message             # Use userdata.my_message

# End states (always null)
transitions:
  succeeded: null
```

---

## Need Help?

1. Look at existing examples in `hsr_task_sm/ros/config/challenges/`
2. Check the state test results
3. Read the ROS logs: `rostopic echo /rosout`
4. Ask your team!

Happy coding! 🤖

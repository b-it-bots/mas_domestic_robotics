# mdr_actions

## Table of contents

1. [Summary](#summary)
2. [Organisation](#organisation)
3. [Dependencies](#dependencies)
4. [Existing actions](#existing-actions)
    1. [mdr_manipulation_actions](#mdr_manipulation_actions)
    2. [mdr_navigation_actions](#mdr_navigation_actions)
    3. [mdr_perception_actions](#mdr_perception_actions)
    4. [mdr_speech_actions](#mdr_speech_actions)

## Summary

The `mdr_actions` metapackage contains a collection of actions that can be composed for executing robot tasks.

The following points apply to all actions in this metapackage:
* Each action is implemented using a fault-tolerant state machine (the `ActionSMBase` in `mas_execution_manager`)
* The action is exposed through an `actionlib` server
* A dedicated action client allows invoking the action through a planning system such as `ROSPlan`; the action client also takes care of updating a robot's knowledge base after a successful execution

## Organisation

The metapackage is split into the following packages:
* [`mdr_manipulation_actions`](mdr_manipulation_actions): Actions that primarily involve manipulating objects
* [`mdr_navigation_actions`](mdr_navigation_actions): Actions for moving a robot around
* [`mdr_perception_actions`](mdr_perception_actions): A set of actions for perceiving various aspects of a robot's environment
* [`mdr_speech_actions`](mdr_speech_actions): A collection of actions for human-robot interaction through speech

## Dependencies

* `mas_knowledge_base`
* `mas_execution_manager`
* `mdr_rosplan_interface`

## Existing actions

### mdr_manipulation_actions

* `mdr_move_arm_action`: A generic action for moving a robot's arm (under a single-arm assumption)
* `mdr_pickup_action`: An action for picking an object from a specified pose
* `mdr_place_action`: A counterpart to `mdr_pickup_action` that allows placing an object at a specified pose

### mdr_navigation_actions

* `mdr_enter_door_action`: Allows a robot to wait for a door to open and then enter the door
* `mdr_follow_person`: (unmaintained) An action for following people around
* `mdr_move_base_action`: A generic action for moving a robot's base; ensures that the manipulator is in a safe configuration before moving
* `mdr_move_forward_action`: Moves a robot's base forward (or backward) with a given speed and for a specified amount of time
* `mdr_turn_base_to_action`: Rotates a robot's base in place

### mdr_perception_actions

* `mdr_perceive_plane_action`: Detects planes in a robot's view and any objects on them
* `mdr_detect_person`: Detects faces in a given image
* `mdr_find_people`: Finds the poses of any people that a robot can see
* `mdr_gender_recognition`: Recognises the gender of previously detected faces in an image
* `mdr_recognize_emotion_action`: Recognises the emotions of previously detected faces in an image
* `mdr_find_object_action`: (under development) Finds the location of a given object by combining navigation, perception and any previous knowledge about the object that a robot might have

### mdr_speech_actions

* `mdr_answer_action`: Provides an answer to a given question
* `mdr_ask_action`: Asks a question corresponding to a given triggering phrase
* `mdr_introduce_self_action`: A simple action for providing basic information about a robot
* `mdr_listen_action`: An action using which a robot can wait for spoken input
* `mdr_process_speech_command_action`: Processes a given robot speech command

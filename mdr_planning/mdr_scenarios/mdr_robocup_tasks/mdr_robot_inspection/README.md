# Robot Inspection Test

## Known Issues / Todo's

Mention the current limitations or issues for the package.
- [ ] Speakers and microphone are not working at the moment
- [ ] The tray is triggering the emergency stop after moving

## Introduction

This scenario is used for the robot inspection test in order to be able to participate during RoboCup@Home competitions.


## Topics

### Published
* `/sound/say (std_msgs/String)`: Phrases said by Jenny during the test, including her own introduction.

### Subscribed to
* `/mcr_perception/qr_reader/output/ (std_msgs/String)`: The content of QR code message perceived by `zbar_ros`
* `/recognized_speech (std_msgs/String)`: The recognized speech published by the `vocon_speech_recognizer` node

## Services
### Required
* `/mdr_actions/enter_door_server (mdr_enter_door_action/EnterDoorAction)`: Detects a door using the front laser scanner and moves forward for an predefined number of seconds.
* `/mdr_actions/move_base_safe_server (mdr_move_base_safe/MoveBaseSafeAction)`: Moves Jenny with the arm in the home position and the tray down to an specified waypoint.
* `/mdr_actions/introduce_self_action_server (mdr_introduce_self_action/IntroduceSelfAction)`: Jenny introduces herself, stating her name, ocupation and place of residence.
* `/mdr_actions/move_tray_server (mdr_move_tray_action/MoveTrayAction)`: Moves the tray up or down.

## Launch Files

* `robot_inspection.launch:` All required components for the tests are launched in their respective PCs:
  * PC1
    * `mdr_2dnav`
    * `mdr_enter_door_action`
    * `mdr_move_base_safe`
    * `mdr_move_tray_action`
    * `mdr_introduce_self_action`
  * PC2
    * `zbar_ros`
  * PC3
    * `vocon_speech_recognizer`
    * `mdr_question_matching`

## Running the Robot Inspection Test on Jenny
In `PC1`:

```shell
roslaunch mdr_robot_inspection robot_inspection.launch
```

```shell
rosrun mdr_robot_inspection robot_inspection
```

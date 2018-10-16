# mdr_listen_action
This package contains the action `listen`. It is used to be attentive to the environment and wait for user input.

## listen action - characteristics
### goal

### result
string message
string message_type
bool success

### feedback
string status_initialization
string status_wait_for_user_input
string status_process_input
bool error_detected

## How does it work?
The `listen` part is supposed to be the action, which enables the listening mode of the robot. Once it is started, the robot listens to its environment for a certain time. If the node does not receive any input in this time, it will terminate and return a failure. On the other hand if a signal is detected then the input is processed and analyzed.

## How to run the code?

```bash
$ rosrun mdr_listen_action listen
```

## How to run the demo?

```bash
$ rosrun mdr_listen_action listen_client_test
```

# mdr_listen_action
This package contains the action `listen`. It is used to be attentive to the environment and wait for user input.

## listen action - characteristics
### goal
int32 listen_duration

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
The `listen` part is supposed to be the action, which enables the listening mode of the robot. Once it is started, the robot listens to its environment for a certain time. This listening duration is defined as the goal of the action. If the node does not receive any input in this time, it will terminate and return a failure. On the other hand if a signal is detected then the input is processed and analyzed. At the moment does the node receive input by subsrcibing to the node /wait_for_user_input (string msg). Once it receives an input, this input is processed and analyzed. If the user publishes one of the options in either ask.txt or answer.txt (left hand side up to the colons), then the code is able to categorize the input in _question_ and _questionable_statement_.

## How to run the code?

```bash
$ rosrun mdr_listen_action listen
```

## How to run the demo?

```bash
$ rosrun mdr_listen_action listen_client test <listen_duration>
```
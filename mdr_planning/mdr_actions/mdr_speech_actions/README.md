# mdr_speech_actions
This package contains the actions `listen`, `ask` and `answer`. They are used to build human-robot communication.
However at this stage many important skills are not implemented, yet.

## How does it work?
The `listen` part is supposed to be the action, which enables the listening mode of the robot. Once it is started, the robot listens to its environment for a certain time. This listening duration is defined as the goal of the action. If the node does not receive any input in this time, it will terminate and return a failure. On the other hand if a signal is detected then the input is processed and analyzed. At the moment does the node receive input by subsrcibing to the node /wait_for_user_input (string msg). Once it receives an input, this input is processed and analyzed. If the user publishes one of the options in either ask.txt or answer.txt (left hand side up to the colons), then the code is able to categorize the input in _question_ and _questionable_statement_.

The actions `ask` and `answer` receive as a goal a string, which is reffered to as _triggering_statement_ (ask) or _question_ (answer). The node then tries to match a response to the received string by comparing the input with saved statements in ask.txt or answer.txt. If there is a match, the node returns the right side of the corresponding colon. Please note, if you want to reach a match, also copy the _._ or the _?_ at the beginning of the line (see ask.txt and answer.txt).

## How to run the code?
There are two options:

1. Run only one node to check how it behaves (example listen):

    ```
    $ rosrun mdr_speech_actions <node>
    ```
    
2. Run the demo (all nodes together):

    * Open six terminals.

    * Start roscore in one of the terminals.

    * Start a listen_server in the second terminal:
    
    ```
    $ rosrun mdr_speech_actions listen
    ```

    * Start an ask_server in the third terminal:
    
    ```
    $ rosrun mdr_speech_actions ask
    ```

    * Start an answer_server in the fourth terminal:
    
    ```
    $ rosrun mdr_speech_actions answer
    ```

    * Prepare a "message" in the fith terminal without executing the command, yet:
    
    ```
    $ rostopic pub /wait_for_user_input std_msgs/String "data: '?wie heißt du'"
    ```

    * Start the _speech_demo_ file:
    
    ```
    $ rosrun mdr_speech_actions speech_demo.py <duration>
    ```

    * Once the speech_demo is running, execute the command in the terminal containing the _rostopic pub_ command.

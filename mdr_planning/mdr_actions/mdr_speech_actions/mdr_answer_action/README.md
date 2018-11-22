# mdr_answer_action
This package contains the action `answer`. It used to prepare an answer to some message.

## answer action - characteristics
### goal
string question

### result
bool success
string answer_message

### feedback
string status_initialization
string status_match_answer
bool error_detected

## How does it work?
The action `answer` receives as a goal a string, which is referred to as _question_. The node then tries to match a response to the received string by comparing the input with saved statements in _answer.txt_ from the mdr_question_answering package. If there is a match, the node returns the right side of the corresponding colon.

### Format of _answer.txt_
Each line represents one possible conversation. On the left hand side the question is written. On the right hand side of the colon the answer is defined.

Example:
```
what are your hobbies:i like to play music on mas parties
```

## How to run the action?

```bash
$ rosrun mdr_answer_action answer
```

## How to run the demo?

```bash
$ rosrun mdr_answer_action answer_client_test <question>
```

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
The action `answer` receives as a goal a string, which is referred to as _question_. The node then tries to match a response to the received string by comparing the input with saved statements in _answer.txt_. If there is a match, the node returns the right side of the corresponding colon. 

Please note, questions in the _answer.txt_ start with a question mark. For now, this is important if you want to combine the `listen` action together with the `answer` action. The question mark is used as an indicator that the passed message is a question and should be forwarded to the `answer` action.

### Format of _answer.txt_
Each line represents one possible conversation. As mentioned before do the lines in _answer.txt_ start with a _?_. The text, which follows after the punctuation mark and which goes up to the colon, is the question. The text after the colon is the response to it. Following this format, you can add more conversations to the file.

Example:
```
?what are your hobbies:i like to play music on mas parties
```

## How to run the action?

```bash
$ rosrun mdr_answer_action answer
```
    
## How to run the demo?

```bash
$ rosrun mdr_answer_action answer_client_test <question>
```

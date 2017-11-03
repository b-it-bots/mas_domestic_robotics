# mdr_ask_action
This package contains the action `ask`. It used to prepare a question to some message.

## answer action - characteristics
### goal
string triggering_statement

### result
bool success
string ask_message

### feedback
string status_initialization
string status_match_question
bool error_detected


## How does it work?
The action `ask` receives as a goal a string, which is referred to as _triggering_statement_. The node then tries to match a response to the received string by comparing the input with saved statements in _ask.txt_. If there is a match, the node returns the right side of the corresponding colon. 

Please note, questions in the _ask.txt_ start with a dot mark. For now, this is important if you want to combine the `listen` action together with the `ask` action. The dot mark is used as an indicator that the passed message is a questionable statement and should be forwarded to the `ask` action.

### Format of _ask.txt_
Each line represents one possible conversation. As mentioned before do the lines in _ask.txt_ start with a _._. The text, which follows after the punctuation mark and which goes up to the colon, is the triggering_statement. The text after the colon is the response to it. Following this format, you can add more conversations to the file.

Example:
```
.the weather is great today:do you have any plans
```

## How to run the code?

```bash
$ rosrun mdr_ask_action ask
```

## How to run the demo?

```bash
$ rosrun mdr_ask_action ask_client_test <triggering_statement>
```
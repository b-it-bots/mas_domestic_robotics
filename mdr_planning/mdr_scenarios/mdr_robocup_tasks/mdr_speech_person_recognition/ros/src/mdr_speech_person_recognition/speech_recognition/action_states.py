import rospy
import smach
import smach_ros
from std_msgs.msg import String
from actionlib import SimpleActionClient

from mdr_answer_action.msg import AnswerGoal, AnswerAction


def say(publisher, message):
    msg = String()
    msg.data = message
    rospy.loginfo(msg.data)
    publisher.publish(msg)


class RequestOperator(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.say_topic = kwargs.get('say_topic', '/say')
        self.say_pub = rospy.Publisher(self.say_topic, String, queue_size=1)

    def execute(self, userdata):
        # Request operator
        say(self.say_pub, "Who wants to play riddles with me?")
        return 'succeeded'


class ProcessSpeech(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeded', 'failed',
                             'failed_after_retrying'],
                             output_keys=['question', 'source_pos',
                                          'question_count'])

        # rospy.Subscriber(speech_topic, String, queue_size=10, speech_cb)
        # TODO convert the question matcher to an action
        self.question_topic = kwargs.get('question_topic',
                                         '/question_matcher/answer')
        self.say_topic = kwargs.get('say_topic', '/say')
        rospy.Subscriber(self.question_topic, String, self.speech_cb)
        self.number_of_retries = 1
        self.retry_count = 0
        self.question_count = 0
        self.question = None
        self.say_pub = rospy.Publisher(self.say_topic, String, queue_size=1)
        self.localized_sound = False

    def execute(self, userdata):
        # TODO Add patrick's julius listening processing here?
        if self.question and self.question_count <= 5:
            userdata.question = self.question
            userdata.question_count = self.question_count
            return 'succedeed'
        elif self.localized_sound and self.question_count > 5:
            # userdata.source_pos = random_pose
            if self.retry_count == self.number_of_retries:
                rospy.loginfo('Failed to understand')
                return 'failed_after_retrying'
            elif not self.question:
                say(self.say_pub, 'Can you repeat the question, please?')
                self.number_of_retries = 1
            else:
                userdata.question = self.question
                userdata.question_count = self.question_count
                self.number_of_retries = 0
        elif not self.question:
            self.number_of_retries = 1
            return 'failed'
        else:
            rospy.logerr('Something went wrong')

        # TODO Sound localization with HSR, how?
        if self.question_count > 5:
            self.localized_sound = True

    def speech_cb(self, msg):
        # TODO call mdr_question_matcher
        # if not recognized:
        if msg.data == 'Sorry, I was not able to recognize your question!':
            say(self.say_pub, msg.data)
            self.question = None
        else:
            self.question = msg.data
            self.question_count = self.question_count + 1


class AnswerQuestion(smach.State):
    def __init__(self, **kwargs):
        smach.State.__init__(self, outcomes=['succeded', 'failed'],
                             input_keys=['question', 'source_pose'])
        self.timeout = kwargs.get('timeout', 20)
        self.say_topic = kwargs.get('say_topic', '/say')
        self.answer_client = SimpleActionClient('mdr_actions/answer_server',
                                                AnswerAction)
        self.answer_client.wait_for_server()
        self.say_pub = rospy.Publisher(self.say_topic, String, queue_size=1)

    def execute(self, userdata):
        answer_goal = AnswerGoal()
        answer_goal.question = userdata.question
        self.answer_client.send_goal(answer_goal)
        result = self.answer_client.wait_for_result()

        say(self.say_pub, result.answer_message)

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
        smach.State.__init__(self, outcomes=['succeeded', 'failed', 'failed_after_retrying'],
                             output_keys=['question', 'source_pos', 'question_count'])

        # rospy.Subscriber(speech_topic, String, queue_size=10, speech_cb)
        # TODO convert the question matcher to an action
        self.question_topic = kwargs.get('question_topic',
                                         '/question_matcher/answer')
        self.say_topic = kwargs.get('say_topic', '/say')
        rospy.Subscriber(self.question_topic, String, self.speech_cb)
        self.number_of_retries = 1
        self.retry_count = 0
        self.question_count = 0
        self.answer = None
        self.say_pub = rospy.Publisher(self.say_topic, String, queue_size=1)
        self.localized_sound = False

    def execute(self, userdata):
        while self.question_count < 15:
            if self.answer and self.question_count <= 5:
                userdata.question = self.answer
                userdata.question_count = self.question_count
                say(self.say_pub, self.answer)
            elif self.localized_sound and self.question_count > 5:
                if self.retry_count == self.number_of_retries:
                    rospy.loginfo('Failed to understand')
                    return 'failed_after_retrying'
                elif not self.answer:
                    say(self.say_pub, 'Can you repeat the question, please?')
                    self.number_of_retries = 1
                else:
                    userdata.question = self.answer
                    userdata.question_count = self.question_count
                    self.number_of_retries = 0
            elif not self.answer:
                self.number_of_retries = 1
                return 'failed'
            else:
                rospy.logerr('Something went wrong')

            # TODO Sound localization with HSR, how?
            if self.question_count > 5:
                self.localized_sound = True

        return 'succeeded'

    def speech_cb(self, msg):
        if msg.data == 'Sorry, I was not able to recognize your question!':
            say(self.say_pub, msg.data)
            self.answer = None
        else:
            self.answer = msg.data
            self.question_count = self.question_count + 1

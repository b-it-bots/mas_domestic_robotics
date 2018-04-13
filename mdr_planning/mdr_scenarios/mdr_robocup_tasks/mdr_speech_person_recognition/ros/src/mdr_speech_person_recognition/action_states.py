import rospy
import smach
import smach_ros


class Dummy(smach.State):
    def __init__(self, msg="", timeout=5):
        smach.State.__init__(self, outcomes=['succeeded', 'failed'])
        self.timeout = timeout
        self.msg = msg

    def execute(self, userdata):
        rospy.loginfo(self.msg)
        rospy.sleep(self.timeout)  # Just for debugging purposes
        return 'succeeded'


class ListenDummyConcurrent(smach.Concurrence):
    def __init__(self, msg="", timeout=5.0):
        smach.Concurrence.__init__(self,
                                   outcomes=['answer', 'ask_repeat',
                                             'turn_and_answer', 'failed'],
                                   default_outcome='answer',
                                   outcome_map={
                                    'turn_and_answer': {
                                        'LOCALIZE_SOUND':   'succeeded',
                                        'LISTEN': 'succeeded'},
                                    'ask_repeat': {
                                        'LOCALIZE_SOUND': 'succeeded',
                                        'LISTEN': 'failed'},
                                    'answer': {
                                        'LOCALIZE_SOUND': 'failed',
                                        'LISTEN': 'succeeded'},
                                    'failed': {
                                        'LOCALIZE_SOUND': 'failed',
                                        'LISTEN': 'failed'}},
                                   output_keys=['question_out'])
        self.timeout = timeout

        with self:
            # Add states to the container
            smach.Concurrence.add('LOCALIZE_SOUND', Dummy())
            smach.Concurrence.add('LISTEN', Dummy())

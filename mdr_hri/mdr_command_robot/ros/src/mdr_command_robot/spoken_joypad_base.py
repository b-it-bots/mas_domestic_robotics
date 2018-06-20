import rospy

from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MoveBaseDirections(object):
    FORWARD = 'forward'
    BACKWARD = 'backward'
    LEFT = 'left'
    RIGHT = 'right'

class MoveHeadDirections(object):
    UP = 'up'
    DOWN = 'down'
    LEFT = 'left'
    RIGHT = 'right'

class GenericMotionCommands(object):
    STOP = 'stop'

class SpokenJoypadBase(object):
    def __init__(self):
        self.robot_name = rospy.get_param('~robot_name', 'bot')
        self.base_linear_vel = float(rospy.get_param('~base_linear_vel', 0.05))
        self.base_angular_vel = float(rospy.get_param('~base_angular_vel', 0.05))
        self.move_base_keywords = rospy.get_param('~move_base_keywords', ['move', 'go'])
        self.turn_base_keywords = rospy.get_param('~turn_base_keywords', ['turn'])
        self.move_head_keywords = rospy.get_param('~move_head_keywords', ['look'])

        self.recognized_speech_topic = rospy.get_param('~recognized_speech_topic',
                                                       '/recognized_speech')

        self.base_vel_topic = rospy.get_param('~base_vel_topic', '/cmd_vel')

        self.recognized_speech_sub = rospy.Subscriber(self.recognized_speech_topic,
                                                      String,
                                                      self.parse_command)
        self.base_vel_pub = rospy.Publisher(self.base_vel_topic, Twist, queue_size=10, latch=True)

        self.current_base_cmd = Twist()

        self.publish_base_commands = False
        while not rospy.is_shutdown():
            if self.publish_base_commands:
                self.base_vel_pub.publish(self.current_base_cmd)
        self.move_base(GenericMotionCommands.STOP)
        self.turn_base(GenericMotionCommands.STOP)
        self.move_head(GenericMotionCommands.STOP)

    def parse_command(self, msg):
        command = msg.data.lower()

        if command.find(GenericMotionCommands.STOP) != -1:
            self.move_base(GenericMotionCommands.STOP)
            self.turn_base(GenericMotionCommands.STOP)
            self.move_head(GenericMotionCommands.STOP)
            self.publish_base_commands = False
            return

        if command.find(self.robot_name) == -1:
            return

        for move_base_keyword in self.move_base_keywords:
            if command.find(move_base_keyword) != -1:
                if command.find(MoveBaseDirections.FORWARD) != -1:
                    self.move_base(MoveBaseDirections.FORWARD)
                elif command.find(MoveBaseDirections.BACKWARD) != -1:
                    self.move_base(MoveBaseDirections.BACKWARD)
                elif command.find(MoveBaseDirections.LEFT) != -1:
                    self.move_base(MoveBaseDirections.LEFT)
                elif command.find(MoveBaseDirections.RIGHT) != -1:
                    self.move_base(MoveBaseDirections.RIGHT)
                self.publish_base_commands = True

        for turn_base_keyword in self.turn_base_keywords:
            if command.find(turn_base_keyword) != -1:
                if command.find(MoveBaseDirections.LEFT) != -1:
                    self.turn_base(MoveBaseDirections.LEFT)
                elif command.find(MoveBaseDirections.RIGHT) != -1:
                    self.turn_base(MoveBaseDirections.RIGHT)
                self.publish_base_commands = True

        for move_head_keyword in self.move_head_keywords:
            if command.find(move_head_keyword) != -1:
                if command.find(MoveHeadDirections.UP) != -1:
                    self.move_head(MoveHeadDirections.UP)
                elif command.find(MoveHeadDirections.DOWN) != -1:
                    self.move_head(MoveHeadDirections.DOWN)
                elif command.find(MoveHeadDirections.LEFT) != -1:
                    self.move_head(MoveHeadDirections.LEFT)
                elif command.find(MoveHeadDirections.RIGHT) != -1:
                    self.move_head(MoveHeadDirections.RIGHT)

    def move_base(self, command):
        if command == GenericMotionCommands.STOP:
            rospy.loginfo('[JOY_MOVE_BASE] Stopping base motion')
            self.current_base_cmd = Twist()
            self.base_vel_pub.publish(self.current_base_cmd)
            return

        twist_msg = Twist()
        if command == MoveBaseDirections.FORWARD:
            rospy.loginfo('[JOY_MOVE_BASE] Going forward')
            twist_msg.linear.x = self.base_linear_vel
        elif command == MoveBaseDirections.BACKWARD:
            rospy.loginfo('[JOY_MOVE_BASE] Going backward')
            twist_msg.linear.x = -self.base_linear_vel
        elif command == MoveBaseDirections.LEFT:
            rospy.loginfo('[JOY_MOVE_BASE] Going to the left')
            twist_msg.linear.y = self.base_linear_vel
        elif command == MoveBaseDirections.RIGHT:
            rospy.loginfo('[JOY_MOVE_BASE] Going to the right')
            twist_msg.linear.y = -self.base_linear_vel

        self.current_base_cmd = twist_msg

    def turn_base(self, command):
        if command == GenericMotionCommands.STOP:
            rospy.loginfo('[JOY_TURN_BASE] Stopping base rotation')
            self.current_base_cmd = Twist()
            self.base_vel_pub.publish(self.current_base_cmd)
            return

        twist_msg = Twist()
        if command == MoveBaseDirections.LEFT:
            rospy.loginfo('[JOY_TURN_BASE] Turning to the left')
            twist_msg.angular.z = self.base_angular_vel
        elif command == MoveBaseDirections.RIGHT:
            rospy.loginfo('[JOY_TURN_BASE] Turning to the right')
            twist_msg.angular.z = -self.base_angular_vel

        self.current_base_cmd = twist_msg

    def move_head(self, command):
        rospy.loginfo('[JOY_MOVE_HEAD] Ignoring request')

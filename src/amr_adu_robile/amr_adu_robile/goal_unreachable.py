import smach
import threading
from geometry_msgs.msg import Twist


class GoalUnreachable(smach.State):
    """Robot is unable to reach goal, because of obstacles."""

    def __init__(self, node):
        smach.State.__init__(self, outcomes=['goal_unreachable'])
        self.node = node
        self.rlock = threading.RLock()

        # Publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def execute(self, userdata):
        with self.rlock:
            twist = Twist()
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            return 'goal_unreachable'

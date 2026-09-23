import smach
import threading
import numpy as np

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion


class GoalReached(smach.State):
    """Robot reached goal and turns into given pose."""

    def __init__(self, node, theta_goal=0.5*np.pi,  # 0.5*pi -> 90°
                 theta_goal_threshold=0.01, max_angular_velocity=0.1,
                 min_angular_velocity=0.1):
        smach.State.__init__(self, outcomes=[
            'turning_to_given_orientation',
            'orientation_reached'
        ])
        self.node = node
        self.rlock = threading.RLock()
        self.lock = threading.Lock()
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.theta_goal = theta_goal
        self.theta_goal_threshold = theta_goal_threshold
        self.robot_angle = 0.0
        self.max_angular_velocity = max_angular_velocity
        self.min_angular_velocity = min_angular_velocity

        # Subscriber
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # Publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        with self.rlock:
            # Extract yaw angle from quaternion
            quat = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.robot_angle = yaw

    def execute(self, userdata):
        with self.rlock:
            angle_error = self.calculate_angle_error(self.theta_goal, self.robot_angle)
            twist = Twist()

            if abs(angle_error) < self.theta_goal_threshold:
                # Stop when both position and orientation reached
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                print("Desired orientation reached.")
                return 'orientation_reached'
            # Rotate to desired orientation

            if angle_error > 0:
                twist.angular.z = np.clip(
                    angle_error,
                    self.min_angular_velocity,
                    self.max_angular_velocity
                )
            else:
                twist.angular.z = np.clip(
                    angle_error,
                    -self.max_angular_velocity,
                    -self.min_angular_velocity
                )

            self.cmd_vel_pub.publish(twist)
            return 'turning_to_given_orientation'

    def calculate_angle_error(self, target_angle, robot_angle):
        """Calculate the smallest angle error between robot orientation and target angle."""
        angle_error = target_angle - robot_angle
        # Normalize to [-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        return angle_error

import rclpy
import numpy as np

from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from tf_transformations import euler_from_quaternion


class FrontierBasedExploration:
    """Robot explores environment autonomously driving to the current range limits."""

    def __init__(self, node):
        self.node = node
        self.latest_scan = None
        self.robot_position = np.array([0.0 0.0])
        self.robot_angle = 0.0
        self.already_visited = None
        self.next_goals = None

        # Subscribers

        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.scan_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        # Publisher

        self.poses_pub = self.node.create_publisher(
            Pose,
            '/estimated_pose',
            10
        )

        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel_pub',
            10
        )

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y

        # Extract yaw angle from quaternion
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_angle = yaw

    def scan_callback(self, msg):
        """Store latest laser scan data."""
        self.latest_scan = msg

    def explore_environment(self):
        # check laser scan and odometry data
        # store already visited poses
        # reach next goal at range limit
        # calculate new goals at next range limit until whole environment is done
        # publish already visited poses continously for SLAM component to map
        

def main(args=None):
    rclpy.init()

    frontier_exploration = FrontierBasedExploration()
    executor = rclpy.get_global_executor()
    executor.add_node(frontier_exploration)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        frontier_exploration.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
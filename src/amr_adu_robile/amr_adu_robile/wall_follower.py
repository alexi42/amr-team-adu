import rclpy
import numpy as np

from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from tf_transformations import euler_from_quaternion

# Code inspired by https://github.com/HBRS-AMR/Robile/blob/main/robile_navigation/robile_navigation_demo/ros/scripts/wall_follower.py


class WallFollwerExploration:
    """
    Robot explores environment autonomously by driving along the walls 
    and then exploring the unknown cells in the occupancy grid map.
    """

    def __init__(self, node):
        self.node = node
        self.latest_scan = None
        self.robot_position = np.array([0.0, 0.0])
        self.robot_angle = 0.0
        self.already_visited = None
        self.next_goals = None
        self.threshold_dist_wall = 1.0
        self.max_linear_vel = 0.3
        self.angular_velocity = 0.3

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
        self.latest_scan_cartesian = self.process_data(np.array(msg))

    def process_data(self, scan):
        """Calculate latest scan measurements to cartesian coorinates."""
        scan_in_cartesian = []
        for i, range_val in enumerate(scan.ranges):
            # Skip invalid readings
            if range_val < scan.range_min or range_val > scan.range_max or :
                continue

            # Calculate angle
            angle = scan.angle_min + i * scan.angle_increment

            # Convert to cartesian coordinates in base_link frame
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            scan_in_cartesian.append(np.array([x, y]))
        return scan_in_cartesian


    def explore_environment(self):
        # check laser scan and odometry data
        # store already visited poses
        # reach next goal at range limit
        # calculate new goals at next range limit until whole environment is done
        # publish already visited poses continously for SLAM component to map
        if self.latest_scan is None:
            return

        current_robot_position = self.robot_position
        next_goal = self.next_goals[0]
        twist = Twist()
        while len(self.next_goals) > 0:
            if np.array_equal(current_robot_position, next_goal):
                del self.next_goals[0]
                self.already_visited = np.append(self.already_visited, current_robot_position)
                print("Next goal was reached.")
            else:
                twist = self.drive_to_next_goal(next_goal)
                self.cmd_vel_pub.publish(twist)
        print("Environment completely explored!")

    def drive_to_next_goal(self, next_goal, twist):
        # check robot's current position
        # check where the next goal is
        # calculate 
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        return twist


def main(args=None):
    rclpy.init()

    frontier_exploration = WallFollwerExploration()
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
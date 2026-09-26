import rclpy
import numpy as np
from rclpy.node import Node
from .pot_field_path_planning import PotentialFieldPathPlanner

from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point, Twist
from tf_transformations import euler_from_quaternion

# Code inspired by https://github.com/HBRS-AMR/Robile/blob/main/robile_navigation/robile_navigation_demo/ros/scripts/wall_follower.py


class WallFollowerExploration(Node):
    """
    Robot explores environment autonomously by driving along the walls 
    and then exploring the unknown cells in the occupancy grid map.
    """

    def __init__(self):
        super().__init__('wall_follower_exploration')
        self.latest_scan = None
        self.robot_position = np.array([0.0, 0.0])
        self.robot_angle = 0.0
        self.already_visited = None
        self.next_goals = None
        self.threshold_dist_wall = 1.0
        self.max_linear_vel = 0.3
        self.angular_velocity = 0.3

        # Subscribers

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        # Publisher

        self.poses_pub = self.create_publisher(
            Pose,
            '/estimated_pose',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel_pub',
            10
        )

        # Control loop timer
        self.timer = self.create_timer(
            0.1,  # 10 Hz
            self.explore_environment,
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
        self.latest_scan_cartesian = self.convert_scan_to_cartesian_coordinates(msg)


    def convert_scan_to_cartesian_coordinates(self, scan):
        """Calculate latest scan measurements to cartesian coorinates."""
        scan_in_cartesian = []
        for i, range_val in enumerate(scan.ranges):
            # Skip invalid readings
            if not (scan.range_min < range_val < scan.range_max):
                continue

            # Calculate angle
            angle = scan.angle_min + i * scan.angle_increment

            # Convert to cartesian coordinates in base_link frame
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)
            scan_in_cartesian.append(np.array([x, y]))
        return scan_in_cartesian

    def convert_scan_to_cartesian_coordinates_in_certain_angle(self, scan, min_angle, max_angle):
        """Calculate latest scan measurements to cartesian coorinates in a certain angle view."""
        scan_in_cartesian = []
        for i, r in enumerate(scan.ranges):
            if not (scan.range_min < r < scan.range_max):
                continue

            angle = scan.angle_min + i * scan.angle_increment

            # Include scan measurements from a certain angle
            if min_angle <= angle <= max_angle:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                scan_in_cartesian.append((x, y, r, angle))
        return scan_in_cartesian

    def explore_environment(self):
        # start: rotate until closest point to wall found
        # drive to wall and keep distance
        # turn left and calculate next waypoint -> max range of scan sensor
        # if in front wall, turn left
        # avoid obstacles by driving on their left side around them then return to wall
        # fill occupancy grid map
        # when returned to start cell/known cell use A* to find closest path to next unknown cell
        # continue mapping until A* can't find a path anymore
        if self.latest_scan is None:
            return
        planner = None
        if self.latest_scan_cartesian is None:
            return
        wall_found = self.find_wall(self.latest_scan_cartesian)
        if wall_found:
            print("Wall found")
            next_goal = self.get_right_wall_target(self.latest_scan)
            planner = PotentialFieldPathPlanner(q_goal=next_goal)
        else:
            print("Couldn't find closest wall :(")
            self.move_around()

    def find_wall(self, latest_scan_cartesian):
        # If points are farther away than 5cm, they don't belong to the same wall
        max_gap = 0.05
        min_points = 5
        # At least five scan readings
        if len(latest_scan_cartesian) < 5:
            return False

        # Distance between neighbouring points in scan order
        gaps = np.linalg.norm(np.diff(latest_scan_cartesian, axis=0), axis=1)

        # Count how long the closest-neighbour run is
        current_run = 1
        best_run = 1

        for gap in gaps:
            if gap <= max_gap:
                current_run += 1
                best_run = max(best_run, current_run)
            else:
                current_run = 1

        # If a long enough cluster exists, it is likely a wall
        return best_run >= min_points

    def get_right_wall_target(self, scan):
        """
        Returns the closest wall point and the driving target point for a right-wall follower.
        The robot tries to keep self.threshold_dist_wall meters from the wall.
        """
        if scan is None:
            return None

        # Right side of the robot in LaserScan frame: [-90°, 0°]
        right_min = -np.pi / 2.0
        right_max = 0.0

        candidates = self.convert_scan_to_cartesian_coordinates_in_certain_angle(scan, right_min, right_max)

        if not candidates:
            return None

        # Closest point to the wall
        closest_angle = min(candidates[3])

        # Desired wall-following clearance
        target_dist = self.threshold_dist_wall

        # Keep the same wall angle, but move to the target clearance
        target_x = target_dist * np.cos(closest_angle)
        target_y = target_dist * np.sin(closest_angle)

        target_point = np.array([target_x, target_y], dtype=float)

        return target_point

    def move_around(self, scan_cartesian):
        """Robot moves around if there is no wall in the current field of view."""
        twist = Twist()
        while not self.find_wall(scan_cartesian):
            twist.linear.x = 0.1
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            print("Found wall! :)")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)

    wall_follower_exploration = WallFollowerExploration()
    executor = rclpy.get_global_executor()
    executor.add_node(wall_follower_exploration)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        wall_follower_exploration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

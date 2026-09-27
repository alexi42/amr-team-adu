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

        # Potential field path planner
        self.planner = None
        self.current_goal = np.array([0.0, 0.0])
        self.goal_tolerance = 0.2

        # Store in which mode the robot currently is
        self.follow = False
        self.turn_left = False
        self.turn_right = False
        self.set_mode()

        # Set thresholds for turning around corners
        self.front_obstacle_distance = 0.45
        self.right_wall_lost_distance = 1.2
        self.wall_reacquired_distance = 0.9

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
        if self.latest_scan is None or self.planner is None:
            return

        # Drive to the closest point and follow the wall
        if np.array_equal(self.current_goal, np.array([0.0, 0.0])):
            wall_point_base = self.get_right_wall_target(self.latest_scan)

            if wall_point_base is None:
                print("No wall on the right side detected :(")
                print("Starting to move around to find wall...")
                self.move_around(self.latest_scan_cartesian)
                return

            self.current_goal = self.base_point_to_odom(
                wall_point_base, self.robot_angle, self.robot_position)
            self.planner.set_goal(self.current_goal)

            print("Next goal: ", self.current_goal)

        # if not self.planner.goal_reached():
        #     return
        # weitermachen

        front_distance = self.get_min_distance(
                    self.latest_scan, -np.pi / 6.0, np.pi / 6.0
                )

        # The robot finds an obstacle ahead and turns left
        if front_distance is not None and front_distance < self.threshold_dist_wall:
            self.set_mode(turn_left=True)
            self.current_goal = self.get_turn_left_goal()
            self.planner.set_goal(self.current_goal)

            print("Corner detected, turning left.")
            return

        right_distance = self.get_min_distance(
            self.latest_scan, -np.pi / 2.0, 0.0
        )

        # The right wall disappeared and the robot turns right until it can keep the distance to the wall again.
        if right_distance is not None and right_distance > self.threshold_dist_wall:
            self.set_mode(turn_right=True)
            self.current_goal = self.get_turn_right_goal(distance=self.threshold_dist_wall)
            self.planner.set_goal(self.current_goal)
            print("Wall disappeared, turning right to find wall again.")
            return
        self.set_mode(follow=True)

        next_point_base = self.get_max_range_point(self.latest_scan)

        if next_point_base is None:
            self.current_goal = None
            return

        self.current_goal = self.base_point_to_odom(
            next_point_base, self.robot_angle, self.robot_position)
        self.planner.set_goal(self.current_goal)

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
        right_min = -np.pi / 2.0
        right_max = 0.0

        candidates = (
            self.convert_scan_to_cartesian_coordinates_in_certain_angle(
                scan,
                right_min,
                right_max,
            )
        )

        if not candidates:
            return None

        # Closest point to the wall
        closest_x, closest_y, distance, closest_angle = min(
            candidates,
            key=lambda point: point[2],
        )

        # Desired wall-following clearance
        target_dist = self.threshold_dist_wall

        # Keep the same wall angle, but move to the target clearance
        target_x = (distance - target_dist) * np.cos(closest_angle)
        target_y = (distance - target_dist) * np.sin(closest_angle)

        return np.array([target_x, target_y])

    def get_max_range_point(self, scan):
        """Return the farthest valid scan point in base_link coordinates."""
        points = (
            self.convert_scan_to_cartesian_coordinates_in_certain_angle(
                scan, -np.pi / 2.0, 0.0
            )
        )

        if not points:
            return None

        x, y, distance, angle = max(
            points,
            key=lambda point: point[2],
        )

        return np.array([x, y])

    def get_min_distance(self, scan, min_angle, max_angle):
        points = (
            self.convert_scan_to_cartesian_coordinates_in_certain_angle(
                scan, min_angle, max_angle)
        )

        if not points:
            return None

        return min(point[2] for point in points)

    def get_turn_left_goal(self, distance=0.8):
        """Return a point in front-left of the robot, in odom coordinates."""
        point_base = np.array([distance, 0.8])

        return self.base_point_to_odom(
            point_base,
            self.robot_angle,
            self.robot_position,
        )

    def get_turn_right_goal(self, distance):
        """Return a point in front-right of the robot, in odom coordinates."""
        point_base = np.array([distance, -0.4])

        return self.base_point_to_odom(
            point_base,
            self.robot_angle,
            self.robot_position,
        )

    def set_mode(self, follow=False, turn_left=False, turn_right=False):
        """Set exactly one navigation mode."""
        self.follow = follow
        self.turn_left = turn_left
        self.turn_right = turn_right

    def move_around(self, scan_cartesian):
        """Robot moves around if there is no wall in the current field of view."""
        twist = Twist()
        if not self.find_wall(scan_cartesian):
            twist.linear.x = 0.1
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            print("Found wall! :)")
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)

    def base_point_to_odom(self, point, robot_angle, robot_position):
        """Convert a point from base_link coordinates to odom coordinates."""
        cos_angle = np.cos(robot_angle)
        sin_angle = np.sin(robot_angle)

        rotation = np.array([
            [cos_angle, -sin_angle],
            [sin_angle, cos_angle],
        ])

        return robot_position + rotation @ point

def main(args=None):
    rclpy.init(args=args)

    wall_follower_exploration = WallFollowerExploration()
    planner = PotentialFieldPathPlanner()
    wall_follower_exploration.planner = planner
    executor = rclpy.get_global_executor()
    executor.add_node(wall_follower_exploration)
    executor.add_node(planner)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        wall_follower_exploration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

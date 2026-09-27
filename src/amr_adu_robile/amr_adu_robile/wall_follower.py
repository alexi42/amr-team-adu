import rclpy
import numpy as np
import subprocess
from .a_star_algorithm import GridCell
from .conversion_script import convert_grid_coordinates_to_world, convert_world_coordinates_to_grid

from rclpy.node import Node
from .pot_field_path_planning_previous_code import PotentialFieldPathPlanner
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, Point, Twist
from tf_transformations import euler_from_quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

# Code inspired by https://github.com/HBRS-AMR/Robile/blob/main/robile_navigation/robile_navigation_demo/ros/scripts/wall_follower.py

ROW = 300
COL = 300
RESOLUTION = 0.05
START = None

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
        self.threshold_dist_wall = 1.0
        self.unknown_cells = None

        # Potential field path planner
        self.planner = None
        self.current_goal = None
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

        # Occupancy grid map
        self.occupancy_grid_map = None
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

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
            qos_profile_sensor_data
        )

        # Publisher

        self.poses_pub = self.create_publisher(
            Pose,
            '/estimated_pose',
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        # Control loop timer

        self.timer = self.create_timer(
            0.1,  # 10 Hz
            self.explore_environment,
        )

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        global START
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y

        # Initialize START from first odometry reading
        if START is None:
            START = (
                self.robot_position[0] - (ROW * RESOLUTION) / 2.0,
                self.robot_position[1] - (COL * RESOLUTION) / 2.0,
            )

        # Extract yaw angle from quaternion
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_angle = yaw

    def scan_callback(self, msg):
        """Store latest laser scan data."""
        self.latest_scan = msg
        self.latest_scan_cartesian = self.convert_scan_to_cartesian_coordinates(msg)

        self.update_occupancy_grid(msg, self.robot_position, self.robot_angle)

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
        """Robot explores environment autonomously."""
        if self.latest_scan is None or self.planner is None:
            return

        if self.occupancy_grid_map is None:
            self.occupancy_grid_map = np.full((ROW, COL), -1, dtype=np.int8)

        coverage = self.get_map_coverage()
        if coverage >= 0.95:
            # Robot has covered enough of the map
            self.stop_robot()
            self.set_mode()
            self.current_goal = None
            self.exploration_done_callback()
            print("Mapping complete!")
            return
        elif coverage >= 0.80:
            # Use A* algorithm to find the way to the first unknown cell
            unknown_cols, unknown_rows = np.where(
                self.occupancy_grid_map == -1
            )
            self.unknown_cells = np.column_stack((unknown_cols, unknown_rows))

            if self.unknown_cells is not None:
                dest = convert_grid_coordinates_to_world(self.unknown_cells[0], START, RESOLUTION)
                print("Destination in grid cell: ", self.unknown_cells[0])
                print("Destination in world", dest)
                way_points = GridCell(ROW, COL, RESOLUTION).a_star_search(
                    self.occupancy_grid_map,
                    self.robot_position,
                    dest,
                    START
                )
                if way_points is not None:
                    print("waypoints are not none.")
                    for wp in way_points:
                        self.current_goal = wp
                        if self.planner.goal_reached():
                            del self.unknown_cells[0]
                            return
                else:
                    self.stop_robot()
                    self.set_mode()
                    self.current_goal = None
                    self.exploration_done_callback()
                    print("Can't calculate a path to the next unknown cell.")
            else:
                self.stop_robot()
                self.set_mode()
                self.current_goal = None
                self.exploration_done_callback()
                print("No unknown cells remain. Mapping complete.")
        else:
            # Robot still needs to explore the environment
            # Drive to the closest point and follow the wall
            if self.current_goal is None:
                wall_point_base = self.get_right_wall_target(self.latest_scan)

                if wall_point_base is None:
                    print("No wall on the right side detected :(")
                    print("Starting to move around to find wall...")
                    self.move_around(self.latest_scan_cartesian)
                    return

                self.current_goal = self.base_point_to_odom(
                    wall_point_base, self.robot_angle, self.robot_position)
                self.planner.set_goal(self.current_goal)

            if not self.planner.goal_reached():
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
                    if front_distance is not None and front_distance < self.threshold_dist_wall:
                        self.set_mode(turn_left=True)
                        self.current_goal = self.get_turn_left_goal()
                        self.planner.set_goal(self.current_goal)
                        print("Too close to wall. Turning away")
                        return
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
            print("Next goal: ", self.current_goal)
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
                scan, -np.pi / 3.0, 0.0
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
            print("Found wall! :)")
        else:
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

    def transform_to_world(self, robot_position, robot_angle, point_base):
        """Transform a point from base_link frame to world  frame."""
        # Rotate by robot_angle (positive rotation)
        cos_a = np.cos(robot_angle)
        sin_a = np.sin(robot_angle)

        rotation_matrix = np.array([
            [cos_a, -sin_a],
            [sin_a, cos_a]
        ])

        # Apply rotation then translation
        point_world = rotation_matrix @ point_base + robot_position
        return point_world


    def publish_occupancy_grid_map(self, occupancy_grid_map):
        if START is None:
            return

        origin_point = Point()
        origin_point.x = float(START[0])
        origin_point.y = float(START[1])

        origin_pose = Pose()
        origin_pose.position = origin_point
        origin_pose.orientation.w = 1.0

        map_meta_data_msg = MapMetaData()
        map_meta_data_msg.resolution = RESOLUTION
        map_meta_data_msg.width = COL
        map_meta_data_msg.height = ROW
        map_meta_data_msg.origin = origin_pose

        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_grid_msg.header.frame_id = 'odom'
        occupancy_grid_msg.info = map_meta_data_msg
        grid_one_dim = [int(i) for row in occupancy_grid_map for i in row]
        occupancy_grid_msg.data = grid_one_dim
        self.occupancy_grid_pub.publish(occupancy_grid_msg)

    def update_occupancy_grid(self, scan, robot_position, robot_angle):
        """Mark visible cells as free and detected endpoints as occupied."""
        if self.occupancy_grid_map is None:
            self.occupancy_grid_map = np.full((COL, ROW), -1, dtype=np.int8)

        robot_col, robot_row = convert_world_coordinates_to_grid(
            robot_position, START, RESOLUTION,
        )

        if robot_row is None or robot_col is None:
            return

        for index, range_value in enumerate(scan.ranges):
            if np.isnan(range_value) or np.isinf(range_value):
                continue

            angle = scan.angle_min + index * scan.angle_increment

            # A max-range beam has no confirmed occupied endpoint.
            has_obstacle = (
                scan.range_min <= range_value < scan.range_max
            )

            distance = min(float(range_value), float(scan.range_max))

            endpoint_base = np.array([
                distance * np.cos(angle),
                distance * np.sin(angle),
            ])

            endpoint_world = self.transform_to_world(
                robot_position, robot_angle, endpoint_base,
            )

            end_col, end_row = (
                convert_world_coordinates_to_grid(
                    endpoint_world,
                    START,
                    RESOLUTION
                )
            )

            if end_row is None or end_col is None:
                continue

            # Mark every cell along the laser ray as free.
            ray_cells = self.bresenham_cells(
                robot_row, robot_col, end_row, end_col
            )

            for row, col in ray_cells[:-1]:
                if 0 <= row < ROW and 0 <= col < COL:
                    # Do not erase a previously detected obstacle.
                    if self.occupancy_grid_map[col, row] != 100:
                        self.occupancy_grid_map[col, row] = 0

            # Only mark the endpoint occupied when the beam returned before range_max.
            if has_obstacle:
                if 0 <= end_row < ROW and 0 <= end_col < COL:
                    self.occupancy_grid_map[end_col, end_row] = 100

        self.publish_occupancy_grid_map(self.occupancy_grid_map)

    def bresenham_cells(self, start_row, start_col, end_row, end_col):
        """Return grid cells on the line between two grid coordinates."""
        cells = []

        delta_row = abs(end_row - start_row)
        delta_col = abs(end_col - start_col)

        step_row = 1 if start_row < end_row else -1
        step_col = 1 if start_col < end_col else -1

        error = delta_col - delta_row
        row = start_row
        col = start_col

        while True:
            cells.append((row, col))

            if row == end_row and col == end_col:
                break

            double_error = 2 * error

            if double_error > -delta_row:
                error -= delta_row
                col += step_col

            if double_error < delta_col:
                error += delta_col
                row += step_row

        return cells

    def get_map_coverage(self):
        if self.occupancy_grid_map is None:
            return 0.0
        known = self.occupancy_grid_map != -1
        rows, cols = np.where(known)

        if (rows.size or cols.size) == 0:
            return 0.0

        min_row, max_row = rows.min(), rows.max()
        min_col, max_col = cols.min(), cols.max()

        explored_area = self.occupancy_grid_map[
            min_row:max_row + 1,
            min_col:max_col + 1
        ]

        known_cells = np.count_nonzero(explored_area != -1)
        total_cells = explored_area.size

        if known_cells > 0 or total_cells > 0:
            return known_cells / total_cells
        else:
            return 0.0

    def stop_robot(self):
        self.planner.set_goal(None)
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def exploration_done_callback(self):
        subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', f'../amr-team-adu/maps/simulation-closed-walls',
            '-t', 'map',
        ], check=True)

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

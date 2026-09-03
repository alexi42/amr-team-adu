import rclpy
import smach
import tf2_ros
import numpy as np
import threading
import heapq
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor

"""
Adapted code for A*-algorithm found on https://www.geeksforgeeks.org/python/a-search-algorithm-in-python/
"""

# Grid size
ROW = 400
COL = 400


class GridCell():
    """
    Class implements A* algorithm
    """
    def __init__(self):
        # row index
        self.parent_i = 0
        # column index
        self.parent_j = 0
        # cost of the cell g+h
        self.f = float('inf')
        # cost from start to this cell
        self.g = float('inf')
        # cost from this cell to destination
        self.h = 0

    # check if the provided cell is valid
    def is_valid(row, col):
        return (row>=0) and (row < ROW) and (col>=0) and (col < COL)

    # check if the given cell is free
    def is_available(grid, row, col):
        return grid[row][col] == 1

    # check if the provided cell is the destination
    def is_destination(row, col, dest):
        return row == dest[0] and col == dest[1]

    # calculate heuristic value as euclidean distance
    def calculate_h_value(row, col, dest):
        return ((row - dest[0]) ** 2 + (col - dest[1]) ** 2) ** 0.5

    # trace the path from the start to the destination
    def trace_path(cell_details, dest):
        # print("The path is")
        path = []
        row = dest[0]
        col = dest[1]

        # Trace the path from destination to source using parent cells
        while not (cell_details[row][col].parent_i == row and cell_details[row][col].parent_j == col):
            path.append((row, col))
            temp_row = cell_details[row][col].parent_i
            temp_col = cell_details[row][col].parent_j
            row = temp_row
            col = temp_col

        # Add the source cell to the path
        path.append((row, col))
        # Reverse the path to get the path from source to destination
        path.reverse()

    def a_star_search(self, grid, src, dest):
        # check if source and destination are valid
        if not self.is_valid(src[0], src[1]) or not self.is_valid(dest[0], dest[1]):
            print("Source or destination is invalid.")
            return

        # check if we are already at destination
        if self.is_destination(src[0], src[1], dest):
            print("We are lready at destination.")
            return

        # Initilize the visited cells
        closed_list = [[False for _ in range(COL)] for _ in range(ROW)]
        # Initialize the details of each cell
        cell_details = [[GridCell() for _ in range(COL)] for _ in range(ROW)]

        # Initialize the start cell details
        i = src[0]
        j = src[1]
        cell_details[i][j].f = 0
        cell_details[i][j].g = 0
        cell_details[i][j].h = 0
        cell_details[i][j].parent_i = i
        cell_details[i][j].parent_j = j

        # Initialize the open list (cells to be visited) with the start cell
        open_list = []
        heapq.heappush(open_list, (0.0, i, j))

        # Initialize the flag for whether destination is found
        found_dest = False

        # Main loop of A* search algorithm
        while len(open_list) > 0:
            # Pop the cell with the smallest f value from the open list
            p = heapq.heappop(open_list)

            # Mark the cell as visited
            i = p[1]
            j = p[2]
            closed_list[i][j] = True

            # For each direction, check the successors
            directions = [
                (0, 1), (0, -1), (1, 0), (-1, 0),
                (1, 1), (1, -1), (-1, 1), (-1, -1)
            ]
            for dir in directions:
                new_i = i + dir[0]
                new_j = j + dir[1]

                # If the successor is valid, unblocked, and not visited
                if self.is_valid(new_i, new_j) and self.is_available(grid, new_i, new_j) and not closed_list[new_i][new_j]:
                    # If the successor is the destination
                    if self.is_destination(new_i, new_j, dest):
                        # Set the parent of the destination cell
                        cell_details[new_i][new_j].parent_i = i
                        cell_details[new_i][new_j].parent_j = j
                        print("The destination cell is found")
                        # Trace and print the path from source to destination
                        self.trace_path(cell_details, dest)
                        found_dest = True
                        return
                    else:
                        # Calculate the new f, g, and h values
                        g_new = cell_details[i][j].g + 1.0
                        h_new = self.calculate_h_value(new_i, new_j, dest)
                        f_new = g_new + h_new

                        # If the cell is not in the open list or the new f value is smaller
                        if cell_details[new_i][new_j].f == float('inf') or cell_details[new_i][new_j].f > f_new:
                            # Add the cell to the open list
                            heapq.heappush(open_list, (f_new, new_i, new_j))
                            # Update the cell details
                            cell_details[new_i][new_j].f = f_new
                            cell_details[new_i][new_j].g = g_new
                            cell_details[new_i][new_j].h = h_new
                            cell_details[new_i][new_j].parent_i = i
                            cell_details[new_i][new_j].parent_j = j

        # If the destination is not found after visiting all cells
        if not found_dest:
            print("Failed to find the destination cell")

class CreateWaypoints(smach.State):
    """
    Creating waypoints at the start.
    Updating them if robot encountered obstacle.
    """

    def __init__(self, node, q_goal=np.array([4.0, 10.0])):
        smach.State.__init__(self, outcomes=[
            'driving_to_goal'
        ])
        self.node = node
        self.q_goal = q_goal
        self.lock = threading.Lock()
        self.robot_position = np.array([-2.0, -3.5])

        # Get robots current position
        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        with self.lock:
            self.robot_position[0] = msg.pose.pose.position.x
            self.robot_position[1] = msg.pose.pose.position.y

            # Extract yaw angle from quaternion
            quat = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.robot_angle = yaw

    def execute(self, userdata):
        # creating/updating waypoints
        current_position = FollowWaypoints.odom_sub()
        waypoints = GridCell.a_star_search(
            grid=[0][0],
            src=self.odom_sub,
            dest=self.q_goal
        )
        return 'driving_to_goal'


class FollowWaypoints(smach.State):
    """
    State to follow waypoints.
    """
    def __init__(self, node, q_goal=np.array([4.0, 10.0]), theta_goal=-1.0,
                 goal_distance_threshold=0.1, goal_angle_threshold=0.1,
                 k_a=0.9, k_r=0.7, rho_0=0.8,
                 max_linear_velocity=0.5, max_angular_velocity=0.8):
        smach.State.__init__(self, outcomes=[
            'driving_to_goal',
            'obstacle_encountered',
            'goal_reached'
        ])
        self.node = node

        # Goal parameters
        self.q_goal = q_goal
        self.theta_goal = theta_goal
        self.goal_distance_threshold = goal_distance_threshold
        self.goal_angle_threshold = goal_angle_threshold

        # Potential field parameters
        self.k_a = k_a  # Attractive force gain
        self.k_r = k_r  # Repulsive force gain
        self.rho_0 = rho_0  # Threshold distance for obstacle influence

        # Velocity parameters
        self.max_linear_velocity = max_linear_velocity  # m/s
        self.max_angular_velocity = max_angular_velocity  # rad/s
        self.linear_gain = 0.8  # scale factor from force magnitude to linear velocity

        # Robot state
        self.robot_position = np.array([-2.0, -3.5])
        self.robot_angle = 0.0
        self.latest_scan = None
        self.path_waypoints = None
        self.lock = threading.Lock()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Subscribers
        self.scan_sub = self.node.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.odom_sub = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        self.path_sub = self.node.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Control loop timer
        self.timer = self.node.create_timer(
            0.1,  # 10 Hz
            self.control_loop,
        )

    def scan_callback(self, msg):
        """Store latest laser scan data."""
        with self.lock:
            self.latest_scan = msg

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        with self.lock:
            self.robot_position[0] = msg.pose.pose.position.x
            self.robot_position[1] = msg.pose.pose.position.y

            # Extract yaw angle from quaternion
            quat = msg.pose.pose.orientation
            _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
            self.robot_angle = yaw

    def path_callback(self, msg):
            """Store latest path data."""
            with self.lock:
                self.path_waypoints = msg

    def control_loop(self):
        """Compute and publish velocity commands."""
        with self.lock:
            if self.latest_scan is None:
                # wait for first scan to arrive
                return

            self.q_goal = self.path_waypoints[0]

            # Otherwise run potential-field based control
            obstacles = self.convert_scan_to_obstacles(self.latest_scan)

            # Calculate forces in base_link frame
            q_base = np.array([0.0, 0.0])  # origin in base_link frame
            q_goal_base = self.transform_to_base_link(self.q_goal)

            attractive_force = self.calculate_attractive_force(q_base, q_goal_base)
            repulsive_force = self.calculate_repulsive_force(q_base, obstacles)

            total_force = attractive_force + repulsive_force
            force_magnitude = np.linalg.norm(total_force)

            twist = Twist()
            if force_magnitude > 1e-6:
                force_direction = total_force / force_magnitude
                linear_vel = np.clip(force_magnitude * self.linear_gain, 0,
                                     self.max_linear_velocity)
                twist.linear.x = force_direction[0] * linear_vel
                angle_to_force = np.arctan2(force_direction[1], force_direction[0])
                twist.angular.z = np.clip(angle_to_force,
                                          -self.max_angular_velocity,
                                          self.max_angular_velocity)

            self.cmd_vel_pub.publish(twist)

    def convert_scan_to_obstacles(self, scan):
        """
        Convert laser scan data to obstacle positions in base_link frame.
        Returns list of obstacle positions as (x, y) in base_link coordinates.
        """

        obstacles = []

        for i, range_val in enumerate(scan.ranges):
            # Skip invalid readings
            if range_val < scan.range_min or range_val > scan.range_max:
                continue

            # Skip very distant readings (noise)
            if range_val > self.rho_0 * 2:
                continue

            # Calculate angle
            angle = scan.angle_min + i * scan.angle_increment

            # Convert to cartesian coordinates in base_link frame
            x = range_val * np.cos(angle)
            y = range_val * np.sin(angle)

            obstacles.append(np.array([x, y]))

        return obstacles

    def transform_to_base_link(self, point_odom):
        """Transform a point from odom frame to base_link frame."""
        # Translate to robot position
        relative_pos = point_odom - self.robot_position

        # Rotate by -robot_angle
        cos_a = np.cos(-self.robot_angle)
        sin_a = np.sin(-self.robot_angle)

        rotation_matrix = np.array([
            [cos_a, -sin_a],
            [sin_a, cos_a]
        ])

        point_base = rotation_matrix @ relative_pos
        return point_base

    def calculate_attractive_force(self, q, q_goal):
        """
        Calculate attractive force towards a provided goal point.

        q: current robot position (x, y)
        q_goal: goal position (x, y) in the same frame as q
        Returns: attractive force
        """
        eps = 1e-6
        direction = q_goal - q
        distance = np.linalg.norm(direction)
        if distance < eps:
            return np.array([0.0, 0.0])
        return self.k_a * direction / distance

    def calculate_repulsive_force(self, q, obstacles):
        """
        Calculate total repulsive force from all obstacles.

        q: current robot position in base_link (should be near origin)
        obstacles: list of obstacle positions in base_link
        Returns: repulsive force
        """
        repulsive_force = np.array([0.0, 0.0])

        eps = 1e-6
        for obstacle_pos in obstacles:
            diff = q - obstacle_pos
            distance = np.linalg.norm(diff)

            # ignore obstacles outside influence radius or too-close values
            if distance >= self.rho_0 or distance < eps:
                continue

            term1 = 1.0 / distance - 1.0 / self.rho_0
            term2 = 1.0 / (distance ** 2)
            direction = diff / distance

            # Sum produces one resultant direction and magnitude of repulsion
            repulsive_force += self.k_r * term1 * term2 * direction
        return repulsive_force

    def check_if_waypoint_in_obstacle(self):
        """Check if there is an obstacle in the next waypoint."""
        epsilon = (0.1, 0.1, 0.1)
        if self.path_waypoints[0] - self.convert_scan_to_obstacles(self.latest_scan) < epsilon:
            return False
        return True

    def execute(self, userdata):
        current_robot_position = self.robot_position
        next_waypoint_pose = self.path_waypoints[1]
        epsilon = (0.01, 0.01, 0.01)
        if next_waypoint_pose - current_robot_position < epsilon:
            del self.path_waypoints[0]
            if len(self.path_waypoints) == 0:
                self.get_logger().info('Goal reached. Stopping robot.')
                return 'goal_reached'
            if self.check_if_waypoint_in_obstacle():
                return 'obstacle_encountered'
            return 'driving_to_goal'


class GoalReached(smach.State):
    """Robot reached goal and turns into given pose."""

    def __init__(self, node, theta_goal=-1.0, theta_goal_threshold=0.1):
        smach.State.__init__(self, outcomes=[
            'turning_to_given_orientation',
            'orientation_reached'
        ])
        self.node = node
        self.cmd_vel_pub = self.node.create_publisher(Twist, 'cmd_vel', 10)
        self.theta_goal = theta_goal
        self.theta_goal_threshold = theta_goal_threshold

    def execute(self, userdata):
        angle_error = self.calculate_angle_error(self.theta_goal)
        twist = Twist()
        twist.linear.x = 0.0
        self.cmd_vel_pub.publish(twist)

        if abs(angle_error) < self.theta_goal_threshold:
            # Stop when both position and orientation reached
            self.cmd_vel_pub.publish(twist)
            return 'orientation_reached'
        else:
            # Rotate to desired orientation only
            twist.angular.z = np.clip(
                angle_error,
                self.max_angular_velocity,
                self.max_angular_velocity
            )
            self.cmd_vel_pub.publish(twist)
            return 'turning_to_given_orientation'

    def calculate_angle_error(self, target_angle):
        """Calculate the smallest angle error between robot orientation and target angle."""
        angle_error = target_angle - self.robot_angle
        # Normalize to [-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        return angle_error


def main(args=None):
    """Main function to initialise and execute state machine."""
    rclpy.init(args=args)

    node = rclpy.create_node('state_machine')

    # Define thresholds

    # Create state machine
    sm = smach.StateMachine(outcomes=['orientation_reached', 'error'])

    # Add states to state machine
    with sm:
        smach.StateMachine.add(
            'CREATE WAYPOINTS',
            CreateWaypoints(node),
            transitions={
                'driving_to_goal': 'FOLLOW WAYPOINTS'
            }
        )

        smach.StateMachine.add(
            'FOLLOW WAYPOINTS',
            FollowWaypoints(node),
            transitions={
                'driving_to_goal': 'FOLLOW WAYPOINTS',
                'obstacle_encountered': 'CREATE WAYPOINTS',
                'goal_reached': 'GOAL REACHED'
            }
        )

        smach.StateMachine.add(
            'GOAL REACHED',
            GoalReached(node),
            transitions={
                'turning_to_given_orientation': 'GOAL REACHED',
                'orientation_reached': 'GOAL REACHED'
            }
        )

    # multihreaded approach because sm.execute() blocks the whole thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    # Execute state machine in a separate thread
    state_thread = threading.Thread(target=sm.execute)
    state_thread.start()

    while True:
        executor.spin_once()


if __name__ == '__main__':
    main()

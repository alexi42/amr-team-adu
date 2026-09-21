import smach
import threading
import tf2_ros
import numpy as np
from .conversion_script import transform_to_base_link, transform_to_world, convert_world_coordinates_to_grid, convert_scan_to_obstacles

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import Twist, PoseStamped
from tf_transformations import euler_from_quaternion


class FollowWaypoints(smach.State):
    """
    State to follow waypoints.
    """
    def __init__(self, node, ROW, COL, RESOLUTION,
                 k_a=0.6, k_r=0.5, rho_0=0.4,
                 max_linear_velocity=0.8, max_angular_velocity=0.5):
        smach.State.__init__(self, outcomes=[
            'driving_to_goal',
            'obstacle_encountered',
            'goal_reached',
            'create_waypoints'
        ])
        self.node = node

        # Goal parameters
        self.q_goal = None

        # Potential field parameters
        self.k_a = k_a  # Attractive force gain
        self.k_r = k_r  # Repulsive force gain
        self.rho_0 = rho_0  # Threshold distance for obstacle influence

        # Velocity parameters
        self.max_linear_velocity = max_linear_velocity  # m/s
        self.max_angular_velocity = max_angular_velocity  # rad/s
        self.linear_gain = 0.8  # scale factor from force magnitude to linear velocity

        # Robot state
        self.robot_position = np.array([0.0, 0.0])
        self.robot_angle = 0.0
        self.latest_scan = None
        self.path_waypoints = None
        self.rlock = threading.RLock()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)

        # Obstacles
        self.obstacles = None

        # Occupancy grid parameters
        self.ROW = ROW
        self.COL = COL
        self.RESOLUTION = RESOLUTION

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

        self.occupancy_grid_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.occupancy_grid_callback,
            10
        )

        # Publisher

        self.trajectory_pub = self.node.create_publisher(
            Path,
            '/robot_trajectory',
            10
        )

        self.trajectory_msg = Path()
        self.trajectory_msg.header.frame_id = 'odom'

        self.last_trajectory_position = None

        self.cmd_vel_pub = self.node.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.local_plan_pub = self.node.create_publisher(
            Path,
            '/local_plan',
            10
        )

    def scan_callback(self, msg):
        """Store latest laser scan data."""
        self.latest_scan = msg

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y

        # Initialize START from first odometry reading to handle floating-point precision
        if self.node.START is None:

            self.node.START = (
                self.robot_position[0] - (self.ROW // 2) * self.RESOLUTION,
                self.robot_position[1] - (self.COL // 2) * self.RESOLUTION
            )

        # Extract yaw angle from quaternion
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_angle = yaw

        current_position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y
        ])

        # Do not add thousands of almost identical points
        if (
            self.last_trajectory_position is None
            or np.linalg.norm(
                current_position - self.last_trajectory_position
            ) > 0.02
        ):
            pose_stamped = PoseStamped()

            pose_stamped.header.frame_id = 'odom'
            pose_stamped.header.stamp = msg.header.stamp

            pose_stamped.pose.position.x = msg.pose.pose.position.x
            pose_stamped.pose.position.y = msg.pose.pose.position.y
            pose_stamped.pose.position.z = msg.pose.pose.position.z

            pose_stamped.pose.orientation = msg.pose.pose.orientation

            self.trajectory_msg.header.stamp = msg.header.stamp
            self.trajectory_msg.poses.append(pose_stamped)

            self.trajectory_pub.publish(self.trajectory_msg)

            self.last_trajectory_position = current_position.copy()

    def path_callback(self, msg):
        """Store path poses as two-dimensional waypoint coordinates."""
        self.path_waypoints = [
            np.array([
                pose.pose.position.x,
                pose.pose.position.y,
            ])
            for pose in msg.poses
        ]
        # Drop the first waypoints only when they exist and the path is long enough.
        while len(self.path_waypoints) > 2:
            if np.linalg.norm(self.path_waypoints[0] - self.robot_position) > 0.15:
                break
            del self.path_waypoints[0]

    def occupancy_grid_callback(self, msg):
        self.occupancy_grid_sub = msg

    def publish_local_plan(self, obstacles, horizon=3.0, dt=0.1):
        """Predict robot motion for the next few seconds and publish it."""
        if self.q_goal is None:
            return

        # Simulated robot pose in odom frame
        x = float(self.robot_position[0])
        y = float(self.robot_position[1])
        theta = float(self.robot_angle)

        # Current laser obstacles converted once into odom/world frame
        obstacles_world = [
            transform_to_world(self, obstacle)
            for obstacle in obstacles
        ]

        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        path_msg.header.stamp = self.node.get_clock().now().to_msg()

        steps = int(horizon / dt)

        for _ in range(steps):

            # Add predicted position to RViz path
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

            # Stop prediction if goal is reached
            if np.linalg.norm(
                self.q_goal - np.array([x, y])
            ) < 0.15:
                break

            # -------------------------------
            # Goal -> simulated base_link
            # -------------------------------

            dx = self.q_goal[0] - x
            dy = self.q_goal[1] - y

            cos_t = np.cos(theta)
            sin_t = np.sin(theta)

            q_goal_base = np.array([
                cos_t * dx + sin_t * dy,
                -sin_t * dx + cos_t * dy
            ])

            # -------------------------------
            # Obstacles -> simulated base_link
            # -------------------------------

            obstacles_base = []

            for obstacle_world in obstacles_world:

                dx_obs = obstacle_world[0] - x
                dy_obs = obstacle_world[1] - y

                obstacle_base = np.array([
                    cos_t * dx_obs + sin_t * dy_obs,
                    -sin_t * dx_obs + cos_t * dy_obs
                ])

                obstacles_base.append(obstacle_base)

            # -------------------------------
            # Same potential-field controller
            # -------------------------------

            q_base = np.array([0.0, 0.0])

            attractive_force = self.calculate_attractive_force(
                q_base,
                q_goal_base
            )

            repulsive_force = self.calculate_repulsive_force(
                q_base,
                obstacles_base
            )

            total_force = attractive_force + repulsive_force

            force_magnitude = np.linalg.norm(total_force)

            if force_magnitude < 1e-6:
                break

            force_direction = total_force / force_magnitude

            linear_vel = np.clip(
                force_magnitude * self.linear_gain,
                0,
                self.max_linear_velocity
            )

            v = force_direction[0] * linear_vel

            angle_to_force = np.arctan2(
                force_direction[1],
                force_direction[0]
            )

            omega = np.clip(
                angle_to_force,
                -self.max_angular_velocity,
                self.max_angular_velocity
            )

            # -------------------------------
            # Predict differential-drive motion
            # -------------------------------

            x += v * np.cos(theta) * dt
            y += v * np.sin(theta) * dt
            theta += omega * dt

            theta = np.arctan2(
                np.sin(theta),
                np.cos(theta)
            )

        self.local_plan_pub.publish(path_msg)

    def control_loop(self):
        """Compute and publish velocity commands."""
        if self.latest_scan is None or not self.path_waypoints:
            # Wait for both a scan and a non-empty path.
            return 'create_waypoints'

        self.q_goal = self.path_waypoints[0]

        # Otherwise run potential-field based control
        self.obstacles = convert_scan_to_obstacles(self.latest_scan)

        self.publish_local_plan(self.obstacles)

        # Calculate forces in base_link frame
        q_base = np.array([0.0, 0.0])  # origin in base_link frame
        q_goal_base = transform_to_base_link(self, self.q_goal)

        attractive_force = self.calculate_attractive_force(q_base, q_goal_base)
        repulsive_force = self.calculate_repulsive_force(q_base, self.obstacles)

        total_force = attractive_force + repulsive_force
        force_magnitude = np.linalg.norm(total_force)

        twist = Twist()
        if force_magnitude > 1e-6:
            force_direction = total_force / force_magnitude
            linear_vel = np.clip(force_magnitude * self.linear_gain, 0,
                                    self.max_linear_velocity)
            twist.linear.x = force_direction[0] * linear_vel
            twist.linear.y = force_direction[1] * linear_vel / 3
            angle_to_force = np.arctan2(force_direction[1], force_direction[0])
            twist.angular.z = np.clip(angle_to_force,
                                        -self.max_angular_velocity,
                                        self.max_angular_velocity)

        self.cmd_vel_pub.publish(twist)

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

    def execute(self, userdata):
        with self.rlock:
            current_robot_position = self.robot_position
            if not self.path_waypoints:
                print("waypoints empty")
                return 'create_waypoints'
            next_waypoint = self.path_waypoints[0]
            epsilon = 0.15

            if self.obstacles is not None and self.path_waypoints is not None:
                for wp in self.path_waypoints:
                    for obstacle in self.obstacles:
                        # wp_base_link = transform_to_base_link(self, wp)
                        wp_grid = convert_world_coordinates_to_grid(wp, self.node.START, self.RESOLUTION)
                        obstacle_world = transform_to_world(self, obstacle)
                        obstacle_grid = convert_world_coordinates_to_grid(obstacle_world, self.node.START, self.RESOLUTION)
                        if wp_grid != (None, None) and obstacle_grid != (None, None) and wp_grid == obstacle_grid:
                            print('Obstacle was encountered')
                            return 'obstacle_encountered'
            twist = Twist()
            if np.linalg.norm(next_waypoint - current_robot_position) < epsilon:
                print("Driving to next waypoint")
                if self.path_waypoints:
                    print("Delete closest waypoint")
                    del self.path_waypoints[0]
                if len(self.path_waypoints) == 0:
                    print("Waypoints are empty")
                    twist.linear.x = 0.0
                    twist.linear.y = 0.0
                    self.cmd_vel_pub.publish(twist)
                    print('Goal reached. Stopping robot.')
                    return 'goal_reached'
            self.control_loop()
            return 'driving_to_goal'

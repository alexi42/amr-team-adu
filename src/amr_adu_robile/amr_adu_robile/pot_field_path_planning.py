import rclpy
import numpy as np
import tf2_ros
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
import threading


class PotentialFieldPathPlanner(Node):
    """
    ROS2 Node implementing potential field-based path planning.
    Subscribes to /scan (laser scan) and /odom (odometry).
    Publishes to /cmd_vel (velocity commands).
    """
    def __init__(self, q_goal=np.array([4.0, 10.0]), theta_goal=-1.0,
                 goal_distance_threshold=0.1, goal_angle_threshold=0.1,
                 k_a=0.9, k_r=0.7, rho_0=0.8,
                 max_linear_velocity=0.5, max_angular_velocity=0.8):
        super().__init__('amr_adu_robile')
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
        self.lock = threading.Lock()

        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10,
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Control loop timer
        self.timer = self.create_timer(
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

    def transform_force_to_odom(self, force_base):
        """
        Transform force from base_link to odom frame.

        force_base: force vector in base_link frame
        Returns: force vector in odom frame
        """
        # Rotation matrix from base_link to odom
        cos_a = np.cos(self.robot_angle)
        sin_a = np.sin(self.robot_angle)

        rotation_matrix = np.array([
            [cos_a, -sin_a],
            [sin_a, cos_a]
        ])

        force_odom = rotation_matrix @ force_base
        return force_odom

    def control_loop(self):
        """Compute and publish velocity commands."""
        with self.lock:
            if self.latest_scan is None:
                # wait for first scan to arrive
                return

            # Current position in odom frame
            q = self.robot_position.copy()

            # Debug: log pose and remaining distance
            distance_to_goal = np.linalg.norm(self.q_goal - q)
            angle_error = self.calculate_angle_error(self.theta_goal)

            # If near the goal position, handle orientation exclusively
            if distance_to_goal < self.goal_distance_threshold:
                twist = Twist()
                if abs(angle_error) < self.goal_angle_threshold:
                    # Stop when both position and orientation reached
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info('Goal reached. Stopping robot.')
                else:
                    # Rotate to desired orientation only
                    twist.angular.z = np.clip(angle_error,
                                              -self.max_angular_velocity,
                                              self.max_angular_velocity)
                    self.cmd_vel_pub.publish(twist)
                return

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

    def calculate_angle_error(self, target_angle):
        """Calculate the smallest angle error between robot orientation and target angle."""
        angle_error = target_angle - self.robot_angle
        # Normalize to [-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        return angle_error

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


def main(args=None):
    """Initialize ROS2 node and start the potential field path planner."""
    rclpy.init(args=args)

    planner = PotentialFieldPathPlanner()

    # Use multithreaded executor for concurrent subscription handling
    executor = MultiThreadedExecutor()
    executor.add_node(planner)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

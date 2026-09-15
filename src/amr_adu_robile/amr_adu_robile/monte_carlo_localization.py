import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from rclpy.executors import MultiThreadedExecutor
import threading

rng = np.random.default_rng()

class ParticleFilter(Node):
    def __init__(self, max_linear_velocity=0.5, max_angular_velocity=0.1):
        super().__init__('particle_filter')


        self.latest_scan = None
        self.robot_position = None
        self.robot_angle = 0.0
        self.lock = threading.Lock()

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_back,
            10
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

    def initialize_particles(num_particles, map_limits):
        return rng.integers(low=0, high=map_limits, size=num_particles)

    def update_belief(x):
        pass

    def calculate_sensor_model(cond, x):
        # step one: calculate likelihood model
        # step two: assigning weights to the particles
        pass

    def resampling():
        pass

    def measurement_update():
        pass

    def send_success_message():
        pass

    def get_picture_size(path):
        return cv2.imread(path).shape[:2]

    def control_loop():
        pass


def main(args=None):
    particle_filter = ParticleFilter()
    path = '../../maps/closed_walls_monte_carlo.pgm'
    position_reached = False

    rclpy.init(args=args)
    node = rclpy.create_node('monte_carlo')
    # get map size
    map_limits = particle_filter.get_picture_size(path)

    particles = particle_filter.initialize_particles(100, map_limits)

    while not position_reached:
        

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
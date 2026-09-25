
from pathlib import Path
import numpy as np
import yaml
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.parameter import Parameter
from rclpy.time import Time
from rclpy.duration import Duration

from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Pose, PoseArray, TransformStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt

from tf_transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
)
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformException



class ParticleFilter(Node):

    def __init__(self):
        super().__init__('particle_filter')

        self.set_parameters([
            Parameter('use_sim_time', value=True)
        ])          

        self.declare_parameter(
            'map_yaml',
            '../amr-team-adu/maps/lab_c-069.yaml'
            )

        self.declare_parameter('num_particles', 300)

        map_path = self.get_parameter('map_yaml').value

        self.num_particles = self.get_parameter('num_particles').value


        if not map_path:
            raise ValueError('Provide the map_yaml parameter.')

        self.rng = np.random.default_rng()
        self.particles = None
        self.last_odom = None

        # For frame transmission
        self.last_odom_stamp = None
        self.odom_frame = 'odom'
        self.base_frame = None

        # Allow RViz to receive the map even if it opens later.
        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        # Subscribers

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile_sensor_data
        )

        self.last_scan_time = None
        self.scan_updates = 0

        # Publishers

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/map',
            map_qos
        )

        self.particle_pub = self.create_publisher(
            PoseArray,
            '/particle_cloud',
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/estimated_pose',
            10
        )

        # Dynamic map -> odom transformation.
        self.tf_broadcaster = TransformBroadcaster(self)

        # Keep disabled until the pose estimate is verified.
        self.declare_parameter('publish_map_tf', True)

        self.good_estimates = 0

        # Load the saved map, then generate particles.
        self.load_map(map_path)
        self.initialise_particles()

        self.publish_map()
        # Publish both for RViz.
        self.timer = self.create_timer(
            1.0,
            self.publish_particles()
        )

        self.get_logger().info('Particle filter started.')



    def scan_callback(self, scan):
        if self.particles is None or self.base_frame is None:
            return  

        # Process approximately two scans per second.
        scan_time = (
            scan.header.stamp.sec
            + scan.header.stamp.nanosec * 1e-9
        )

        if self.last_scan_time is not None:
            dt = scan_time - self.last_scan_time
            if 0 <= dt < 0.5:
                return

        # Select 24 valid LiDAR beams.
        ranges = np.asarray(scan.ranges)

        valid_indices = np.where(
            np.isfinite(ranges)
            & (ranges >= scan.range_min)
            & (ranges < scan.range_max)
        )[0]

        if len(valid_indices) < 5:
            return

        selected = valid_indices[
            np.linspace(
                0,
                len(valid_indices) - 1,
                min(24, len(valid_indices)),
                dtype=int
            )
        ]

        distances = ranges[selected]
        angles = scan.angle_min + selected * scan.angle_increment

        # Get the LiDAR's position relative to the robot.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                scan.header.frame_id,
                Time()
            )
        except TransformException as exc:
            self.get_logger().warning(f'Laser TF unavailable: {exc}')
            return

        self.last_scan_time = scan_time

        t = tf.transform.translation
        q = tf.transform.rotation

        _, _, laser_yaw = euler_from_quaternion([
            q.x, q.y, q.z, q.w
        ])

        # Calculate the LiDAR position for every particle.
        theta = self.particles[:, 2]

        laser_x = (
            self.particles[:, 0]
            + np.cos(theta) * t.x
            - np.sin(theta) * t.y
        )

        laser_y = (
            self.particles[:, 1]
            + np.sin(theta) * t.x
            + np.cos(theta) * t.y
        )

        # Direction of each LiDAR beam for every particle.
        ray_angles = (
            theta[:, None]
            + laser_yaw
            + angles[None, :]
        )

        # Limit ray casting to 8 metres for this first test.
        max_range = min(float(scan.range_max), 8.0)

        # What would each particle see at its hypothetical pose?
        predicted = self.ray_cast(
            laser_x,
            laser_y,
            ray_angles,
            max_range
        )

        # Compare predictions against the real LiDAR measurements.
        observed = np.minimum(distances, max_range)
        errors = predicted - observed[None, :]

        # Small distance errors produce higher likelihoods.
        sigma = 0.20

        probabilities = (
            0.05
            + 0.95 * np.exp(
                -0.5 * (errors / sigma) ** 2
            )
        )

        # Combine all selected beams to calculate particle weights.
        log_weights = np.sum(np.log(probabilities), axis=1)
        log_weights -= np.max(log_weights)

        weights = np.exp(log_weights)
        weights /= np.sum(weights)



        estimate = self.estimate_pose(weights)

        if estimate is not None:
            self.publish_estimated_pose(
                estimate,
                scan.header.stamp
            )

            if self.get_parameter('publish_map_tf').value:
                self.publish_map_to_odom(estimate)

        # Resample using the calculated particle weights.
        self.resample_particles(weights)

        self.scan_updates += 1

        if self.scan_updates % 10 == 0:
            self.get_logger().info(
                f'Completed {self.scan_updates} LiDAR updates.'
            )

    def ray_cast(self, laser_x, laser_y, ray_angles, max_range):
        # One predicted distance for every particle and LiDAR beam.
        predicted = np.full(ray_angles.shape, max_range)
        active = np.ones(ray_angles.shape, dtype=bool)

        # Direction of every simulated beam.
        direction_x = np.cos(ray_angles)
        direction_y = np.sin(ray_angles)

        ox, oy, map_yaw = self.origin
        c = np.cos(map_yaw)
        s = np.sin(map_yaw)

        # Move along the beams in steps of one map cell.
        for distance in np.arange(
            0.0, max_range, self.resolution
        ):
            if not np.any(active):
                break

            # Current point along each simulated beam.
            x = (
                laser_x[:, None]
                + distance * direction_x
            )
            y = (
                laser_y[:, None]
                + distance * direction_y
            )

            # Convert world coordinates to map-grid coordinates.
            dx = x - ox
            dy = y - oy

            cols = np.floor(
                (c * dx + s * dy) / self.resolution
            ).astype(int)

            rows = np.floor(
                (-s * dx + c * dy) / self.resolution
            ).astype(int)

            inside = (
                (rows >= 0)
                & (rows < self.height)
                & (cols >= 0)
                & (cols < self.width)
            )

            # Clip indices so we can safely access the grid.
            safe_rows = np.clip(rows, 0, self.height - 1)
            safe_cols = np.clip(cols, 0, self.width - 1)

            # Stop each beam at its first occupied cell.
            hit = (
                active
                & inside
                & (self.grid[safe_rows, safe_cols] == 100)
            )

            predicted[hit] = distance
            active[hit] = False

            # Outside the known map: no mapped obstacle detected.
            active[~inside] = False

        return predicted
    
    def resample_particles(self, weights):
        # Standard resampling according to LiDAR weights.
        indices = self.rng.choice(
            self.num_particles,
            size=self.num_particles,
            replace=True,
            p=weights
        )

        self.particles = self.particles[indices].copy()

        # Replace 5% with random recovery particles.
        num_random = int(0.05 * self.num_particles)

        replace_indices = self.rng.choice(
            self.num_particles,
            size=num_random,
            replace=False
        )

        self.particles[replace_indices] = (
            self.sample_random_particles(num_random)
        )

    def estimate_pose(self, weights):
        best_index = np.argmax(weights)
        best_particle = self.particles[best_index]
        dx = self.particles[:, 0] - best_particle[0]
        dy = self.particles[:, 1] - best_particle[1]

        distances = np.hypot(dx, dy)

        # Orientation difference, normalised to [-pi, pi].
        angle_diff = self.particles[:, 2] - best_particle[2]        

        angle_diff = np.arctan2(
            np.sin(angle_diff),
            np.cos(angle_diff)
        )

        # Select nearby particles with similar orientations.
        cluster = (
            (distances < 0.5)
            & (np.abs(angle_diff) < 0.6)
        )

        cluster_weight = np.sum(weights[cluster])

        # Do not estimate a pose from a weak or tiny cluster.
        if np.count_nonzero(cluster) < 5:
            return None

        if cluster_weight < 0.6:
            return None

        poses = self.particles[cluster]

        # Normalise the selected particle weights.
        w = weights[cluster] / cluster_weight

        x = np.sum(w * poses[:, 0])
        y = np.sum(w * poses[:, 1])

        # Circular mean for orientation.
        sin_mean = np.sum(w * np.sin(poses[:, 2]))
        cos_mean = np.sum(w * np.cos(poses[:, 2]))

        theta = np.arctan2(sin_mean, cos_mean)

        # Measure how concentrated this cluster is.
        position_spread = np.sqrt(
            np.sum(
                w * (
                    (poses[:, 0] - x) ** 2
                    + (poses[:, 1] - y) ** 2
                )
            )
        )

        orientation_concentration = np.hypot(
            sin_mean,
            cos_mean
        )

        if position_spread > 0.35:
            return None

        if orientation_concentration < 0.9:
            return None

        return np.array([x, y, theta])

    def publish_estimated_pose(self, estimate, stamp):
        x, y, theta = estimate

        msg = PoseStamped()

        msg.header.frame_id = 'map'
        msg.header.stamp = stamp

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)

        q = quaternion_from_euler(0.0, 0.0, float(theta))

        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pose_pub.publish(msg)


    def publish_map_to_odom(self, estimate):
        if self.last_odom is None or self.last_odom_stamp is None:
            return

        # Robot pose estimated by our particle filter, in map.
        xm, ym, theta_m = estimate

        # The same robot's pose reported by odometry.
        xo, yo, theta_o = self.last_odom

        # Relative rotation: map -> odom.
        yaw = theta_m - theta_o
        yaw = np.arctan2(np.sin(yaw), np.cos(yaw))

        c = np.cos(yaw)
        s = np.sin(yaw)

        # Relative translation from homogeneous transformation.
        tx = xm - (c * xo - s * yo)
        ty = ym - (s * xo + c * yo)

        transform = TransformStamped()

        # Publish slightly into the future so RViz can transform
        # incoming sensor messages between localisation updates.
        transform.header.stamp = (
            Time.from_msg(self.last_odom_stamp)
            + Duration(seconds=1.0)
        ).to_msg()

        transform.header.frame_id = 'map'
        transform.child_frame_id = self.odom_frame

        transform.transform.translation.x = float(tx)
        transform.transform.translation.y = float(ty)
        transform.transform.translation.z = 0.0

        q = quaternion_from_euler(0.0, 0.0, float(yaw))

        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(transform)

    def load_map(self, yaml_path):
        yaml_path = Path(yaml_path).expanduser().resolve()

        with open(yaml_path, 'r') as file:
            config = yaml.safe_load(file)

        # The PGM path is relative to the YAML location.
        image_path = yaml_path.parent / config['image']

        image = np.array(
            Image.open(image_path).convert('L')
        )

        height, width = image.shape

        self.resolution = float(config['resolution'])
        self.origin = config['origin']

        # Convert image brightness to occupancy probability.
        if config.get('negate', 0):
            occupancy = image.astype(float) / 255.0
        else:
            occupancy = 1.0 - image.astype(float) / 255.0

        occupied_threshold = float(config['occupied_thresh'])
        free_threshold = float(config['free_thresh'])

        grid = np.full((height, width), -1, dtype=np.int8)

        grid[occupancy <= free_threshold] = 0
        grid[occupancy >= occupied_threshold] = 100

        # PGM rows start at the top; OccupancyGrid starts at the bottom.
        self.grid = np.flipud(grid)

        self.height, self.width = self.grid.shape
        self.map_frame = 'map'

        self.get_logger().info(
            f'Map loaded: {self.width} x {self.height} cells, '
            f'resolution {self.resolution} m'
        )

        occupied = self.grid == 100

        self.distance_map = distance_transform_edt(
            ~occupied
        ) * self.resolution

    def sample_random_particles(self, count):
        # Find all known free cells in the saved map.
        free_rows, free_cols = np.where(self.grid == 0)

        if len(free_rows) == 0:
            raise ValueError('The map contains no free cells.')

        # Randomly select free cells.
        indices = self.rng.choice(
            len(free_rows),
            size=count,
            replace=True
        )

        rows = free_rows[indices]
        cols = free_cols[indices]

        # Random positions inside the selected cells.
        local_x = (
            cols + self.rng.random(count)
        ) * self.resolution

        local_y = (
            rows + self.rng.random(count)
        ) * self.resolution

        # Convert grid positions into map coordinates.
        origin_x, origin_y, origin_theta = self.origin

        c = np.cos(origin_theta)
        s = np.sin(origin_theta)

        x = origin_x + c * local_x - s * local_y
        y = origin_y + s * local_x + c * local_y

        # Random orientations between -pi and pi.
        theta = self.rng.uniform(-np.pi, np.pi, count)

        return np.column_stack((x, y, theta))

    def initialise_particles(self):
        self.particles = self.sample_random_particles(
            self.num_particles
        )

        self.get_logger().info(
            f'Initialised {self.num_particles} particles.'
        )

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        self.last_odom_stamp = msg.header.stamp
        self.odom_frame = msg.header.frame_id or 'odom'
        self.base_frame = msg.child_frame_id or 'base_footprint'

        _, _, theta = euler_from_quaternion([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        ])

        current_odom = np.array([
            position.x,
            position.y,
            theta
        ])

        if self.last_odom is None:
            self.last_odom = current_odom
            return

        previous = self.last_odom
        self.last_odom = current_odom

        if self.particles is None:
            return

        # World-frame displacement from odometry.
        dx = current_odom[0] - previous[0]
        dy = current_odom[1] - previous[1]

        # Express displacement in the previous robot frame.
        c = np.cos(previous[2])
        s = np.sin(previous[2])

        dx_local = c * dx + s * dy
        dy_local = -s * dx + c * dy

        dtheta = np.arctan2(
            np.sin(current_odom[2] - previous[2]),
            np.cos(current_odom[2] - previous[2])
        )

        distance = np.hypot(dx_local, dy_local)

        # Initial motion-noise parameters.
        position_noise = 0.005 + 0.05 * distance
        angle_noise = 0.005 + 0.05 * abs(dtheta)

        noisy_dx = dx_local + self.rng.normal(
            0.0, position_noise, self.num_particles
        )

        noisy_dy = dy_local + self.rng.normal(
            0.0, position_noise, self.num_particles
        )

        noisy_dtheta = dtheta + self.rng.normal(
            0.0, angle_noise, self.num_particles
        )

        particle_theta = self.particles[:, 2]

        # Apply each motion relative to its particle's orientation.
        self.particles[:, 0] += (
            np.cos(particle_theta) * noisy_dx
            - np.sin(particle_theta) * noisy_dy
        )

        self.particles[:, 1] += (
            np.sin(particle_theta) * noisy_dx
            + np.cos(particle_theta) * noisy_dy
        )

        self.particles[:, 2] += noisy_dtheta

        # Normalise orientations to [-pi, pi].
        self.particles[:, 2] = np.arctan2(
            np.sin(self.particles[:, 2]),
            np.cos(self.particles[:, 2])
        )


    def publish_visualisation(self):
        self.publish_map()
        self.publish_particles()


    def publish_map(self):
        msg = OccupancyGrid()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height

        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])

        q = quaternion_from_euler(
            0.0, 0.0, self.origin[2]
        )

        msg.info.origin.orientation.x = q[0]
        msg.info.origin.orientation.y = q[1]
        msg.info.origin.orientation.z = q[2]
        msg.info.origin.orientation.w = q[3]

        msg.data = self.grid.flatten().tolist()

        self.map_pub.publish(msg)


    def publish_particles(self):
        if self.particles is None:
            return

        msg = PoseArray()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        for x, y, theta in self.particles:
            pose = Pose()

            pose.position.x = float(x)
            pose.position.y = float(y)

            q = quaternion_from_euler(
                0.0, 0.0, float(theta)
            )

            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]

            msg.poses.append(pose)

        self.particle_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = ParticleFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

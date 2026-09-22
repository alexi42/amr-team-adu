import smach
import numpy as np
import threading
from .conversion_script import convert_scan_to_obstacles, convert_grid_coordinates_to_world, convert_world_coordinates_to_grid, transform_to_world, transform_to_base_link
from .a_star_algorithm import GridCell

from nav_msgs.msg import Odometry, Path, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf_transformations import euler_from_quaternion


class CreateWaypoints(smach.State):
    """
    Creating waypoints at the start.
    Updating them if robot encountered obstacle.
    """

    def __init__(self, node, ROW, COL, RESOLUTION,
                 q_goal=np.array([-3.0, 1.2])):
        smach.State.__init__(self, outcomes=[
            'create_waypoints',
            'driving_to_goal'
        ])
        self.node = node
        self.q_goal = q_goal
        self.rlock = threading.RLock()
        self.robot_position = np.array([0.0, 0.0])
        self.waypoints = None
        self.latest_scan = None
        self.occupancy_grid_map = None

        self.node.DESTINATION = q_goal

        # Occupancy grid parameters
        self.ROW = ROW
        self.COL = COL
        self.RESOLUTION = RESOLUTION

        # Subscriber to get robots current position
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

        # Publisher to pass waypoints
        self.path = self.node.create_publisher(
            Path,
            '/path',
            10
        )

        # Publisher for occupancy grid

        self.occupancy_grid = self.node.create_publisher(
            OccupancyGrid,
            '/map',
            10
        )

    def odom_callback(self, msg):
        """Update robot pose from odometry."""
        self.robot_position[0] = msg.pose.pose.position.x
        self.robot_position[1] = msg.pose.pose.position.y

        # Initialize START from first odometry reading to handle floating-point precision
        if self.node.START is None:
            self.node.START = (
                self.robot_position[0] - (self.ROW * self.RESOLUTION) / 2.0,
                self.robot_position[1] - (self.COL * self.RESOLUTION) / 2.0,
            )

        # Extract yaw angle from quaternion
        quat = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.robot_angle = yaw

    def scan_callback(self, msg):
        """Store latest laser scan data."""
        self.latest_scan = msg

    def write_obstacles_into_grid(self):
        if self.node.START is None:
            return np.zeros((self.ROW, self.COL))
        if self.occupancy_grid_map is None:
            self.occupancy_grid_map = np.zeros((self.ROW, self.COL), dtype=np.uint8)
        grid = self.occupancy_grid_map
        if self.latest_scan is not None:
            obstacles_base = convert_scan_to_obstacles(self.latest_scan)
            for obstacle_base in obstacles_base:

                obstacle_world = transform_to_world(self, obstacle_base)

                row, col = convert_world_coordinates_to_grid(
                    obstacle_world,
                    self.node.START,
                    self.RESOLUTION
                )
                if row is None or col is None:
                    continue
                if 0 <= row < self.ROW and 0 <= col < self.COL:
                    grid[row][col] = 1

                    # Also mark neighbour cells as obstructed, staying inside the grid.
                    for d_row in (-1, 0, 1):
                        for d_col in (-1, 0, 1):
                            inflated_row = row + d_row
                            inflated_col = col + d_col
                            if 0 <= inflated_row < self.ROW and 0 <= inflated_col < self.COL:
                                grid[inflated_row][inflated_col] = 1
            self.occupancy_grid_map = grid
        return grid

    def execute(self, userdata):
        """Create waypoints or update them after encountering new obstacle."""
        with self.rlock:
            if self.node.START is None or self.latest_scan is None:
                return 'create_waypoints'

            gridcell = GridCell(ROW=self.ROW, COL=self.COL, RESOLUTION=self.RESOLUTION)
            current_position = self.robot_position

            origin_point = Point()
            origin_point.x = float(self.node.START[0])
            origin_point.y = float(self.node.START[1])

            origin_pose = Pose()
            origin_pose.position = origin_point
            origin_pose.orientation.w = 1.0

            map_meta_data_msg = MapMetaData()
            map_meta_data_msg.resolution = self.RESOLUTION
            map_meta_data_msg.width = self.COL
            map_meta_data_msg.height = self.ROW
            map_meta_data_msg.origin = origin_pose

            occupancy_grid_msg = OccupancyGrid()
            occupancy_grid_msg.info = map_meta_data_msg
            grid_two_dim = self.write_obstacles_into_grid()
            grid_one_dim = [int(i) for row in grid_two_dim for i in row]
            occupancy_grid_msg.data = grid_one_dim
            self.occupancy_grid.publish(occupancy_grid_msg)

            if np.any(grid_two_dim) or np.any(current_position) or self.q_goal is not None:
                self.waypoints = gridcell.a_star_search(
                    grid=grid_two_dim,
                    src=current_position,
                    dest=self.q_goal,
                    start=self.node.START
                )

            if self.waypoints is None:
                return 'create_waypoints'

            waypoints_world = [
                convert_grid_coordinates_to_world(w, start=self.node.START, res=self.RESOLUTION)
                for w in self.waypoints
            ]
            self.waypoints = waypoints_world

            path_msg = Path()
            path_msg.header.frame_id = 'odom'
            path_msg.header.stamp = self.node.get_clock().now().to_msg()

            waypoint_poses = []
            for w in self.waypoints:
                pose_stamped = PoseStamped()

                pose_stamped.header.frame_id = 'odom'
                pose_stamped.header.stamp = path_msg.header.stamp

                pose_stamped.pose.position.x = float(w[0])
                pose_stamped.pose.position.y = float(w[1])
                pose_stamped.pose.position.z = 0.0

                # Valid quaternion
                pose_stamped.pose.orientation.w = 1.0

                waypoint_poses.append(pose_stamped)

            path_msg.poses = waypoint_poses
            self.path.publish(path_msg)
            return 'driving_to_goal'

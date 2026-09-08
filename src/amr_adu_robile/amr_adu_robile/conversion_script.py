"""
Collection of conversion methods.

This script is a collection of needed methods
to convert coordinates and objects into the needed frame.
"""

import numpy as np


def convert_world_coordinates_to_grid(coord, start, res):
    """Convert world coordinates to coordinates in the custom grid."""
    x, y = coord

    if start is None:
        return None, None

    row = int(np.floor((x - start[0]) / res))
    col = int(np.floor((y - start[1]) / res))
    return row, col


def convert_grid_coordinates_to_world(coord, start, res):
    """Convert grid coordinates back to world coordinates."""
    x, y = coord

    if start is None:
        return None, None

    row = x * res + start[0]
    col = y * res + start[1]
    return row, col


def convert_scan_to_obstacles(self):
    """
    Convert laser scan data to obstacle positions in base_link frame.

    Returns list of obstacle positions as (x, y) in base_link coordinates.
    """
    obstacles = []
    scan = self.latest_scan
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

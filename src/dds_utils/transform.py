import numpy as np
from std_msgs.msg import Float64MultiArray


def parse_transform_msg(msg):
    """Parse Float64MultiArray from transformation_matrix topic into (R, t)."""
    transformation_matrix = msg.data
    R = np.array(transformation_matrix[:4]).reshape(2, 2)
    t = np.array(transformation_matrix[4:])
    return R, t


def pack_transform_msg(R, t):
    """Pack 2x2 R and translation t into Float64MultiArray (same layout as before)."""
    transform_msg = Float64MultiArray()
    transform_msg.data = np.concatenate((R.flatten(), t.flatten()))
    return transform_msg


def transform_point(point, R, t, forward=True):
    if R is None:
        return point

    point_xy = np.array([point[0], point[1]])
    if forward:
        new_point_xy = R @ point_xy + t
        new_point_theta = point[2] + np.arctan2(R[1, 0], R[0, 0])
        return np.concatenate((new_point_xy, [new_point_theta]))
    new_point_xy = R.T @ (point_xy - t)
    new_point_theta = point[2] - np.arctan2(R[1, 0], R[0, 0])
    return np.concatenate((new_point_xy, [new_point_theta]))


def transform_points(points, R, t, forward=True):
    if R is None:
        return points

    points_xy = np.array([points[0, :], points[1, :]])
    if forward:
        new_point_xy = R @ points_xy + t
        new_point_theta = points[2, :] + np.arctan2(R[1, 0], R[0, 0])
        return np.concatenate((new_point_xy, new_point_theta))
    new_point_xy = R.T @ (points_xy - t)
    new_point_theta = points[2, :] - np.arctan2(R[1, 0], R[0, 0])
    return np.concatenate((new_point_xy, [new_point_theta]))


class TransformMixin:
    """R/t state from transformation_matrix + transform helpers."""

    def init_transform_state(self):
        self.R = None
        self.t = None

    def transformation_callback(self, msg):
        self.R, self.t = parse_transform_msg(msg)

    def transform_point(self, point, forward=True):
        return transform_point(point, self.R, self.t, forward)

    def transform_points(self, points, forward=True):
        return transform_points(points, self.R, self.t, forward)

    def update_transformation(self, R, t):
        self.R = R
        self.t = t

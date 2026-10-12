import numpy as np
from std_msgs.msg import Float64MultiArray

from .map_transform import build_map_transform, load_known_points, map_transform_from_doc  # noqa: F401

# transformation_matrix layout:
#   legacy:   [R00 R01 R10 R11 tx ty]
#   extended: [R00 R01 R10 R11 tx ty  version  model  smooth  N  src_x0 src_y0 ...  dst_x0 dst_y0 ...]
# The extended form carries the point pairs so every node rebuilds the identical transform.
_LAYOUT_VERSION = 1
_MODEL_CODES = {"rigid": 0, "tps": 1}
_MODEL_NAMES = {v: k for k, v in _MODEL_CODES.items()}


def parse_transform_msg(msg):
    """Parse Float64MultiArray from transformation_matrix topic into a MapTransform (or None)."""
    data = [float(v) for v in msg.data]
    doc = {"R": data[:4], "t": data[4:6], "model": "rigid"}
    if len(data) > 6:
        version, model, smooth, n = data[6:10]
        if int(version) != _LAYOUT_VERSION:
            raise ValueError(f"unsupported transformation_matrix layout version {version}")
        n = int(n)
        doc["model"] = _MODEL_NAMES.get(int(model), "rigid")
        doc["smooth"] = smooth
        doc["src_points"] = data[10:10 + 2 * n]
        doc["dst_points"] = data[10 + 2 * n:10 + 4 * n]
    return map_transform_from_doc(doc)


def pack_transform_msg(map_tf):
    """Pack a MapTransform into Float64MultiArray (extended layout; first 6 values are the rigid R, t)."""
    doc = map_tf.to_doc()
    src = doc.get("src_points", [])
    dst = doc.get("dst_points", [])
    header = [_LAYOUT_VERSION, _MODEL_CODES[doc["model"]], doc.get("smooth", 0.0), len(src) // 2]
    transform_msg = Float64MultiArray()
    transform_msg.data = [float(v) for v in list(doc["R"]) + list(doc["t"]) + header + list(src) + list(dst)]
    return transform_msg


def transform_point(point, map_tf, forward=True):
    if map_tf is None:
        return point
    return map_tf.apply(point, forward)


def transform_points(points, map_tf, forward=True):
    """Transform a 3xN array of [x; y; theta] columns."""
    if map_tf is None:
        return points
    points = np.asarray(points, dtype=float)
    return np.stack([map_tf.apply(points[:, i], forward) for i in range(points.shape[1])], axis=1)


class TransformMixin:
    """MapTransform state from transformation_matrix + transform helpers."""

    def init_transform_state(self):
        self.map_tf = None
        self.R = None
        self.t = None

    def transformation_callback(self, msg):
        self.update_transformation(parse_transform_msg(msg))

    def transform_point(self, point, forward=True):
        return transform_point(point, self.map_tf, forward)

    def transform_points(self, points, forward=True):
        return transform_points(points, self.map_tf, forward)

    def update_transformation(self, map_tf):
        self.map_tf = map_tf
        # Rigid approximation, kept for code that still reads R/t directly.
        self.R = None if map_tf is None else map_tf.R
        self.t = None if map_tf is None else map_tf.t

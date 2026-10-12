"""Unit tests for the vendored map transform and the /transformation_matrix glue (no ROS needed)."""

import os
import sys

import numpy as np
import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "src", "dds_utils"))
import map_transform as mt  # noqa: E402
from script_stubs import Float64MultiArray, RecordingTf, stubbed_imports  # noqa: E402

LOCAL_POINTS = [(0.0, 0.0), (10.0, 0.5), (10.5, 9.0), (0.5, 10.0), (5.0, 5.0), (2.0, 7.5), (8.0, 3.0)]


def _rigid(points, theta=0.7, t=(3.0, -2.0)):
    R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return [tuple(R @ np.array(p) + np.array(t)) for p in points]


def _warped(points):
    """Rigid motion plus a smooth non-rigid bend, like two maps that drifted apart."""
    out = []
    for x, y in _rigid(points):
        out.append((x + 0.3 * np.sin(y / 4.0), y + 0.2 * np.cos(x / 5.0)))
    return out


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _query_poses(n=25, seed=0):
    rng = np.random.default_rng(seed)
    return np.column_stack([rng.uniform(0, 10, n), rng.uniform(0, 10, n), rng.uniform(-np.pi, np.pi, n)])


@pytest.fixture
def glue():
    with stubbed_imports() as g:
        yield g


def test_tps_on_rigid_data_matches_estimate_se2():
    ref = _rigid(LOCAL_POINTS)
    tf = mt.build_map_transform(LOCAL_POINTS, ref, "tps")
    assert tf.model == "tps"
    R, t = mt.estimate_se2(LOCAL_POINTS, ref)
    for pose in _query_poses():
        expected = mt.transform_se2(R, t, pose, True)
        got = tf.apply(pose, True)
        np.testing.assert_allclose(got[:2], expected[:2], atol=1e-6)
        assert abs(_wrap(got[2] - expected[2])) < 1e-6


def test_tps_round_trip_on_warped_data():
    tf = mt.build_map_transform(LOCAL_POINTS, _warped(LOCAL_POINTS), "tps")
    assert tf.model == "tps"
    for pose in _query_poses():
        back = tf.apply(tf.apply(pose, True), False)
        np.testing.assert_allclose(back[:2], pose[:2], atol=1e-6)
        assert abs(_wrap(back[2] - pose[2])) < 1e-6


def test_tps_passes_through_known_points():
    ref = _warped(LOCAL_POINTS)
    tf = mt.build_map_transform(LOCAL_POINTS, ref, "tps")
    np.testing.assert_allclose(tf.apply_xy(np.array(LOCAL_POINTS), True), np.array(ref), atol=1e-6)


def test_three_points_fall_back_to_rigid():
    tf = mt.build_map_transform(LOCAL_POINTS[:3], _warped(LOCAL_POINTS[:3]), "tps")
    assert tf.model == "rigid"
    assert tf.fallback_reason


def test_rigid_mode_is_rigid():
    tf = mt.build_map_transform(LOCAL_POINTS, _warped(LOCAL_POINTS), "rigid")
    assert tf.model == "rigid"
    assert tf.fallback_reason is None


def test_identical_points_give_identity():
    # First agent: entry_exit passes its own tuples as the reference; Initialization JSON gives lists.
    for ref in (list(LOCAL_POINTS), [list(p) for p in LOCAL_POINTS]):
        tf = mt.build_map_transform(LOCAL_POINTS, ref, "tps")
        assert tf.model == "rigid"
        np.testing.assert_array_equal(tf.R, np.identity(2))
        np.testing.assert_array_equal(tf.t, np.zeros(2))


def test_count_mismatch_raises():
    with pytest.raises(ValueError):
        mt.build_map_transform(LOCAL_POINTS, _warped(LOCAL_POINTS)[:-1], "tps")


@pytest.mark.parametrize("mode,n", [("tps", 7), ("rigid", 7), ("tps", 3)])
def test_pack_parse_round_trip_is_exact(glue, mode, n):
    tf = mt.build_map_transform(LOCAL_POINTS[:n], _warped(LOCAL_POINTS[:n]), mode)
    msg = glue.pack_transform_msg(tf)
    assert all(isinstance(v, float) for v in msg.data)
    parsed = glue.parse_transform_msg(Float64MultiArray(msg.data))
    assert parsed.model == tf.model
    for pose in _query_poses(10):
        for forward in (True, False):
            np.testing.assert_array_equal(parsed.apply(pose, forward), tf.apply(pose, forward))
    np.testing.assert_array_equal(parsed.R, tf.R)
    np.testing.assert_array_equal(parsed.t, tf.t)


def test_extended_layout_starts_with_legacy_r_t(glue):
    tf = mt.build_map_transform(LOCAL_POINTS, _warped(LOCAL_POINTS), "tps")
    data = glue.pack_transform_msg(tf).data
    np.testing.assert_array_equal(data[:4], tf.R.flatten())
    np.testing.assert_array_equal(data[4:6], tf.t)
    assert len(data) == 10 + 4 * len(LOCAL_POINTS)


def test_legacy_six_float_message_matches_old_math(glue):
    R, t = mt.estimate_se2(LOCAL_POINTS, _rigid(LOCAL_POINTS))
    parsed = glue.parse_transform_msg(Float64MultiArray(list(R.flatten()) + list(t)))
    assert parsed.model == "rigid"
    for pose in _query_poses(10):
        for forward in (True, False):
            np.testing.assert_array_equal(parsed.apply(pose, forward), mt.transform_se2(R, t, pose, forward))


def test_unknown_layout_version_rejected(glue):
    with pytest.raises(ValueError):
        glue.parse_transform_msg(Float64MultiArray([1, 0, 0, 1, 0, 0, 99, 0, 0, 0]))


def test_mixin_passthrough_without_transform(glue):
    node = glue.TransformMixin()
    node.init_transform_state()
    point = [1.0, 2.0, 0.3]
    assert node.transform_point(point) is point
    assert node.transform_point(point, forward=False) is point


def test_mixin_forwards_direction_and_mirrors_r_t(glue):
    node = glue.TransformMixin()
    node.init_transform_state()
    rec = RecordingTf()
    node.update_transformation(rec)
    assert node.R is rec.R and node.t is rec.t
    node.transform_point([1.0, 2.0, 0.0])
    node.transform_point([1.0, 2.0, 0.0], forward=False)
    assert rec.directions == [True, False]


def test_mixin_callback_parses_message(glue):
    tf = mt.build_map_transform(LOCAL_POINTS, _warped(LOCAL_POINTS), "tps")
    node = glue.TransformMixin()
    node.init_transform_state()
    node.transformation_callback(glue.pack_transform_msg(tf))
    assert node.map_tf.model == "tps"
    np.testing.assert_array_equal(node.transform_point([3.0, 4.0, 0.5]), tf.apply([3.0, 4.0, 0.5]))


def test_mixin_transform_points_3xn(glue):
    tf = mt.build_map_transform(LOCAL_POINTS, _warped(LOCAL_POINTS), "tps")
    node = glue.TransformMixin()
    node.init_transform_state()
    node.update_transformation(tf)
    poses = _query_poses(5).T  # 3xN
    out = node.transform_points(poses)
    assert out.shape == poses.shape
    for i in range(poses.shape[1]):
        np.testing.assert_array_equal(out[:, i], tf.apply(poses[:, i]))

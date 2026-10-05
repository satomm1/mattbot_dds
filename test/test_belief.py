"""Unit tests for dds_utils/belief.py (no ROS needed): python3 -m pytest mattbot_dds/test"""

import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest

# Import belief.py directly so the test doesn't need cyclonedds/rospy (pulled in by dds_utils/__init__).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dds_utils"))
from belief import (  # noqa: E402
    DECAYING,
    HELD,
    UNKNOWN,
    TrackedObject,
    belief_grid,
    belief_to_occupancy,
    object_belief,
    object_state,
    objects_from_ledger,
)

T1, T2 = 10.0, 20.0


@pytest.mark.parametrize(
    "age, expected",
    [(-5.0, 1.0), (0.0, 1.0), (T1, 1.0), (T1 + T2 / 2, 0.5), (T1 + T2, 0.0), (1000.0, 0.0)],
)
def test_belief_curve(age, expected):
    assert object_belief(age, T1, T2) == pytest.approx(expected)


def test_belief_no_decay_period():
    assert object_belief(T1, T1, 0.0) == 1.0
    assert object_belief(T1 + 0.1, T1, 0.0) == 0.0


def lobs(object_id, x, y, width, stamp, observer=3, cls="chair"):
    return SimpleNamespace(object_id=object_id, class_name=cls, observer_id=observer,
                           x=x + 100.0, y=y + 100.0, local_x=x, local_y=y, width=width, stamp=stamp)


def test_objects_from_ledger_groups_across_agents():
    agents = [
        SimpleNamespace(observations=[lobs("a", 1.0, 1.0, 0.4, 5.0), lobs("b", 4.0, 4.0, 0.2, 1.0, cls="cone")]),
        SimpleNamespace(observations=[lobs("a", 1.2, 1.4, 0.6, 9.0, observer=6)]),
    ]
    a, b = objects_from_ledger(agents)
    assert (a.object_id, b.object_id) == ("a", "b")
    assert (a.local_x, a.local_y, a.x, a.y) == pytest.approx((1.1, 1.2, 101.1, 101.2))
    assert a.width == 0.6
    assert (a.first_stamp, a.last_stamp, a.num_observations) == (5.0, 9.0, 2)
    assert a.observer_ids == [3, 6]
    assert b.class_name == "cone" and b.num_observations == 1


def obj(x, y, width, last_stamp, object_id="a"):
    return TrackedObject(object_id, "chair", x, y, x, y, width, last_stamp, last_stamp, 1, [3])


@pytest.mark.parametrize(
    "now, belief, phase, to_decay, to_unknown",
    [
        (0.0, 1.0, HELD, T1, T1 + T2),
        (T1, 1.0, HELD, 0.0, T2),
        (T1 + T2 / 2, 0.5, DECAYING, 0.0, T2 / 2),
        (T1 + T2, 0.0, UNKNOWN, 0.0, 0.0),
        (1000.0, 0.0, UNKNOWN, 0.0, 0.0),
    ],
)
def test_object_state(now, belief, phase, to_decay, to_unknown):
    s = object_state(obj(0.0, 0.0, 0.5, 0.0), now, T1, T2)
    assert s.belief == pytest.approx(belief)
    assert s.phase == phase
    assert s.age == now
    assert (s.time_to_decay, s.time_to_unknown) == pytest.approx((to_decay, to_unknown))


def grid(objects, now=0.0, w=10, h=8, res=1.0, ox=0.0, oy=0.0):
    """objects: [(x, y, width, last_stamp), ...]"""
    states = [object_state(obj(*o), now, T1, T2) for o in objects]
    return belief_grid(states, w, h, res, ox, oy)


def test_prior_is_free():
    g = grid([])
    assert g.shape == (8, 10)
    assert np.all(g == -1.0)


def test_footprint_is_set():
    g = grid([(4.5, 3.5, 2.0, 0.0)])  # covers x in [3.5, 5.5], y in [2.5, 4.5]
    assert np.all(g[2:5, 3:6] == 1.0)
    assert np.sum(g == 1.0) == 9
    assert g[0, 0] == -1.0


def test_small_object_covers_one_cell():
    g = grid([(4.5, 3.5, 0.0, 0.0)])
    assert np.sum(g == 1.0) == 1 and g[3, 4] == 1.0


def test_overlap_takes_max():
    now = T1 + T2 / 2  # first object at 0.5
    g = grid([(4.5, 3.5, 2.0, 0.0), (5.5, 3.5, 0.0, now)], now=now)
    assert g[3, 4] == pytest.approx(0.5)
    assert g[3, 5] == 1.0


def test_decayed_object_is_unknown_not_free():
    g = grid([(4.5, 3.5, 0.0, 0.0)], now=1000.0)
    assert g[3, 4] == 0.0


def test_clipping_and_outside_map():
    g = grid([(0.2, 0.2, 2.0, 0.0), (50.0, 50.0, 1.0, 0.0)])
    assert np.sum(g == 1.0) == 4  # clipped corner footprint; far object ignored
    assert np.all(g[0:2, 0:2] == 1.0)


def test_origin_offset():
    g = grid([(-0.5, 2.5, 0.0, 0.0)], ox=-5.0, oy=0.0, res=0.5)
    assert g[5, 9] == 1.0


def test_occupancy_encoding():
    g = np.array([[-1.0, 0.0, 1.0, 0.5]], dtype=np.float32)
    assert belief_to_occupancy(g) == [0, 50, 100, 75]

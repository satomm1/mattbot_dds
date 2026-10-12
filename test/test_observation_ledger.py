"""Unit tests for the stale sync-response guard in scripts/observation_ledger.py (no ROS needed).

rospy, the message packages and dds_utils (cyclonedds) are stubbed; only the real ledger.py is used.
"""

import importlib
import json
import os
import sys
import time
import types
from unittest import mock

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, "..", "src", "dds_utils"))
import ledger  # noqa: E402


def _stub(name, **attrs):
    mod = types.ModuleType(name)
    for k, v in attrs.items():
        setattr(mod, k, v)
    return mod


@pytest.fixture
def node_module():
    """Import observation_ledger with stubbed ROS / DDS dependencies."""
    any_cls = mock.MagicMock
    dds_utils = _stub(
        "dds_utils",
        INIT_DISCOVERY_GRACE_S=0.0,
        ROS_TOPIC_AGENTS_TO_SUBSCRIBE="/agents_to_subscribe",
        ROS_TOPIC_TRANSFORMATION_MATRIX="/transformation_matrix",
        DdsLogger=lambda _name: mock.MagicMock(),
        GatedResolver=ledger.GatedResolver,
        Observation=ledger.Observation,
        ObservationLedger=ledger.ObservationLedger,
        Removal=ledger.Removal,
        RobotIdError=Exception,
        TransformMixin=object,
        make_obs_id=ledger.make_obs_id,
        require_robot_id_int=lambda: 1,
    )
    stubs = {
        "rospy": mock.MagicMock(),
        "std_msgs": _stub("std_msgs"),
        "std_msgs.msg": _stub("std_msgs.msg", Float64MultiArray=any_cls, Int16MultiArray=any_cls, String=any_cls),
        "mattbot_image_detection": _stub("mattbot_image_detection"),
        "mattbot_image_detection.msg": _stub("mattbot_image_detection.msg", DetectedObject=any_cls),
        "mattbot_dds": _stub("mattbot_dds"),
        "mattbot_dds.msg": _stub("mattbot_dds.msg", Ledger=any_cls, LedgerAgentEntry=any_cls, LedgerObservation=any_cls),
        "mattbot_dds.srv": _stub(
            "mattbot_dds.srv",
            GetLedger=any_cls, GetLedgerResponse=any_cls, RemoveObject=any_cls, RemoveObjectResponse=any_cls,
        ),
        "dds_utils": dds_utils,
    }
    with mock.patch.dict(sys.modules, stubs), mock.patch.object(sys, "path", [os.path.join(HERE, "..", "scripts")] + sys.path):
        sys.modules.pop("observation_ledger", None)
        mod = importlib.import_module("observation_ledger")
        yield mod
        sys.modules.pop("observation_ledger", None)


def make_node(mod, start_wall):
    """An ObservationLedgerNode with just the state sync_response_callback uses (no rospy.init_node)."""
    import threading

    node = mod.ObservationLedgerNode.__new__(mod.ObservationLedgerNode)
    node.start_wall = start_wall
    node.ledger = ledger.ObservationLedger()
    node.resolver = ledger.GatedResolver(0.75)
    node.lock = threading.Lock()
    node.dirty = False
    node.synced = set()
    return node


def response(responder=5, sent_at=None):
    oid = ledger.make_obs_id(responder, 100, 1)
    obs = ledger.Observation(oid, oid, responder, 100, 1, 0.0, "chair", 0.9, 0.5, 1.0, 2.0)
    payload = {"responder": responder, "observations": [obs.to_dict()], "removals": [], "seq_high": {"100": 1}}
    if sent_at is not None:
        payload["sent_at"] = sent_at
    return types.SimpleNamespace(data=json.dumps(payload))


def test_stale_response_is_ignored(node_module):
    start = time.time()
    node = make_node(node_module, start)
    node.sync_response_callback(response(sent_at=int(start) - 60))
    assert node.synced == set()
    assert node.ledger.observations_of(5) == []
    assert not node.dirty


def test_current_response_is_applied(node_module):
    start = time.time()
    node = make_node(node_module, start)
    node.sync_response_callback(response(sent_at=int(start)))
    assert node.synced == {5}
    assert len(node.ledger.observations_of(5)) == 1


def test_response_within_slack_is_applied(node_module):
    # Integer-second envelope timestamps: a reply sent just before start (same/previous second) counts.
    start = time.time()
    node = make_node(node_module, start)
    node.sync_response_callback(response(sent_at=int(start) - 1))
    assert node.synced == {5}


def test_response_without_sent_at_is_applied(node_module):
    node = make_node(node_module, time.time())
    node.sync_response_callback(response())
    assert node.synced == {5}
    assert len(node.ledger.observations_of(5)) == 1

"""Each node sends its data through map_tf in the right direction (no ROS needed).

Outgoing data (published to the swarm / central node) uses forward=True (local -> reference).
Incoming data (goals, position_init, other agents' poses) uses forward=False (reference -> local).
RecordingTf shifts x by +100 forward / -100 inverse, so the asserted outputs prove the transformed value is used.
"""

import json
import os
import sys
import threading
from types import SimpleNamespace
from unittest import mock

import pytest

HERE = os.path.dirname(__file__)
sys.path.insert(0, HERE)
from script_stubs import RecordingTf, import_script, stubbed_imports  # noqa: E402


@pytest.fixture
def load():
    with stubbed_imports():
        yield import_script


def _attach(node):
    node.init_transform_state()
    rec = RecordingTf()
    node.update_transformation(rec)
    return rec


def _reader(*samples):
    return SimpleNamespace(read=lambda: list(samples))


def _pose_msg(x, y):
    return SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=x, y=y)))


# ---------- Incoming (inverse) ----------


def _self_data_listener(mod):
    listener = mod.SelfDataListener.__new__(mod.SelfDataListener)
    listener.my_id, listener.my_id_int = "1", 1
    listener.goal_pub = mock.MagicMock()
    listener.goal_multi_pub = mock.MagicMock()
    listener.init_pub = mock.MagicMock()
    listener._mirror_multi_to_external_goal = False
    listener.db = None
    return listener


def _data_sample(message_type, data, sending_agent=0):
    return SimpleNamespace(sending_agent=sending_agent, message_type=message_type, timestamp=0, data=json.dumps(data))


@pytest.mark.parametrize("message_type", ["goal", "multi_robot_goal"])
def test_own_data_goals_are_inverse(load, message_type):
    mod = load("own_data_subscriber")
    listener = _self_data_listener(mod)
    rec = _attach(listener)

    listener.on_data_available(_reader(_data_sample(message_type, {"x": 150.0, "y": 2.0, "theta": 0.5})))

    assert rec.calls == [((150.0, 2.0, 0.5), False)]
    if message_type == "goal":
        published = listener.goal_pub.publish.call_args[0][0]
        assert (published.x, published.y, published.theta) == (50.0, 2.0, 0.5)
    else:
        published = listener.goal_multi_pub.publish.call_args[0][0]
        assert (published.goal.x, published.goal.y, published.goal.theta) == (50.0, 2.0, 0.5)


def test_own_data_position_init_is_inverse(load):
    mod = load("own_data_subscriber")
    mod.rospy.Time.now.return_value.__sub__.return_value.to_sec.return_value = 0.0
    mod.POSITION_INIT_RECENT_THRESHOLD_S = 5.0
    listener = _self_data_listener(mod)
    rec = _attach(listener)

    listener.on_data_available(_reader(_data_sample("position_init", {"x": 150.0, "y": 2.0, "theta": 0.5})))

    assert rec.calls == [((150.0, 2.0, 0.5), False)]
    assert listener.init_pub.publish.call_args[0][0].pose.pose.position.x == 50.0


def test_location_subscriber_other_agent_pose_is_inverse(load):
    mod = load("location_subscriber")
    listener = mod.LocationListener.__new__(mod.LocationListener)
    listener.my_id_int = 1
    listener.agent_location_publisher = mock.MagicMock()
    rec = _attach(listener)

    listener.on_data_available(_reader(SimpleNamespace(agent_id=2, x=150.0, y=2.0, theta=0.5, static=False)))

    assert rec.calls == [((150.0, 2.0, 0.5), False)]
    assert tuple(listener.locations) == (50.0, 2.0, 0.5)


def test_observation_ledger_local_view_is_inverse(load):
    mod = load("observation_ledger")
    node = mod.ObservationLedgerNode.__new__(mod.ObservationLedgerNode)
    rec = _attach(node)
    node.lock = threading.Lock()
    obs = SimpleNamespace(
        obs_id="o", object_id="o", observer_id=2, session="s", seq=1, stamp=0.0,
        class_name="chair", probability=0.9, width=0.5, x=150.0, y=2.0,
    )
    node.ledger = mock.MagicMock()
    node.ledger.query.return_value = [(2, [obs])]

    node.build_ledger_msg()

    assert rec.calls == [((150.0, 2.0, 0.0), False)]
    assert mod.LedgerObservation.call_args.kwargs["local_x"] == 50.0


# ---------- Outgoing (forward) ----------


def _data_publisher(mod):
    node = mod.DataPublisher.__new__(mod.DataPublisher)
    node._publish_data = mock.MagicMock()
    node.db = None
    return node


def test_data_publisher_detection_is_forward(load):
    mod = load("data_publisher")
    node = _data_publisher(mod)
    rec = _attach(node)
    msg = _pose_msg(1.0, 2.0)

    node.confirmed_object_callback(msg)

    assert rec.calls == [((1.0, 2.0, 0.0), True)]
    assert msg.pose.position.x == 101.0
    assert node._publish_data.call_args[0][0] == "detected_object"


def test_data_publisher_path_is_forward(load):
    mod = load("data_publisher")
    node = _data_publisher(mod)
    rec = _attach(node)
    msg = SimpleNamespace(poses=[_pose_msg(1.0, 2.0), _pose_msg(3.0, 4.0)])

    node.path_callback(msg)

    assert rec.directions == [True, True]
    assert [p.pose.position.x for p in msg.poses] == [101.0, 103.0]


@pytest.mark.parametrize("callback", ["invalid_goal_callback", "voice_goal_callback"])
def test_data_publisher_goal_reports_are_forward(load, callback):
    mod = load("data_publisher")
    node = _data_publisher(mod)
    rec = _attach(node)
    msg = SimpleNamespace(x=1.0, y=2.0, theta=0.5)

    getattr(node, callback)(msg)

    assert rec.calls == [((1.0, 2.0, 0.5), True)]
    assert (msg.x, msg.y, msg.theta) == (101.0, 2.0, 0.5)


def test_data_publisher_patrol_goal_is_forward(load):
    mod = load("data_publisher")
    node = _data_publisher(mod)
    rec = _attach(node)

    node.patrol_goal_callback(SimpleNamespace(x=1.0, y=2.0, theta=0.5))

    assert rec.calls == [((1.0, 2.0, 0.5), True)]
    assert mod.Pose2D.return_value.x == 101.0


def test_observation_ledger_ego_observation_is_forward(load):
    mod = load("observation_ledger")
    node = mod.ObservationLedgerNode.__new__(mod.ObservationLedgerNode)
    rec = _attach(node)
    node.lock = threading.Lock()
    node.my_id, node.session, node.seq = 1, "s", 0
    node.resolver, node.ledger, node.obs_pub = mock.MagicMock(), mock.MagicMock(), mock.MagicMock()
    mod.json = mock.MagicMock()
    msg = SimpleNamespace(pose=_pose_msg(1.0, 2.0).pose, class_name="chair", probability=0.9, width=0.5)

    node.ego_callback(msg)

    assert rec.calls == [((1.0, 2.0, 0.0), True)]
    assert mod.Observation.call_args.kwargs["x"] == 101.0


def _one_loop(mod):
    mod.rospy.is_shutdown.side_effect = [False, True]
    mod.tf.transformations.euler_from_quaternion.return_value = (0.0, 0.0, 0.5)


def test_location_publisher_pose_is_forward(load):
    mod = load("location_publisher")
    node = mod.LocationPublisher.__new__(mod.LocationPublisher)
    rec = _attach(node)
    node.my_id_int, node.is_static = 1, False
    node.trans_listener = mock.MagicMock()
    node.trans_listener.lookupTransform.return_value = ((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    node.location_writer = mock.MagicMock()
    _one_loop(mod)

    node.run()

    assert rec.calls == [((1.0, 2.0, 0.5), True)]
    args = mod.Location.call_args[0]
    assert (args[2], args[3], args[4]) == (101.0, 2.0, 0.5)


def test_heartbeat_pose_is_forward(load):
    mod = load("heartbeat_publisher")
    node = mod.HeartbeatPublisher.__new__(mod.HeartbeatPublisher)
    rec = _attach(node)
    node.my_id_int, node.my_ip, node.mcu_connected = 1, "1.2.3.4", True
    node.trans_listener = mock.MagicMock()
    node.trans_listener.lookupTransform.return_value = ((1.0, 2.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    node.heartbeat_writer = mock.MagicMock()
    _one_loop(mod)

    node.run()

    assert rec.calls == [((1.0, 2.0, 0.5), True)]
    args = mod.Heartbeat.call_args[0]
    assert (args[5], args[6], args[7]) == (101.0, 2.0, 0.5)

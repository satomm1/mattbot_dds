"""Unit tests for dds_utils/ledger.py (no ROS needed): python3 -m pytest mattbot_dds/test"""

import os
import sys

# Import ledger.py directly so the test doesn't need cyclonedds/rospy (pulled in by dds_utils/__init__).
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src", "dds_utils"))
from ledger import GatedResolver, Observation, ObservationLedger, PassThroughResolver, Removal, make_obs_id  # noqa: E402


def obs(observer, seq, stamp=0.0, cls="chair", session=100):
    oid = make_obs_id(observer, session, seq)
    return Observation(oid, oid, observer, session, seq, stamp, cls, 0.9, 0.5, 1.0, 2.0)


def test_add_is_idempotent():
    ledger = ObservationLedger()
    assert ledger.add(obs(6, 1))
    assert not ledger.add(obs(6, 1))
    assert len(ledger.observations_of(6)) == 1


def test_query_sorted_by_agent_then_stamp():
    ledger = ObservationLedger()
    ledger.add(obs(6, 1, stamp=20.0))
    ledger.add(obs(3, 1, stamp=5.0))
    ledger.add(obs(6, 2, stamp=10.0))
    result = ledger.query()
    assert [aid for aid, _ in result] == [3, 6]
    assert [o.stamp for o in result[1][1]] == [10.0, 20.0]


def test_query_filters():
    ledger = ObservationLedger()
    ledger.add(obs(6, 1, stamp=1.0, cls="chair"))
    ledger.add(obs(6, 2, stamp=2.0, cls="cone"))
    ledger.add(obs(3, 1, stamp=3.0, cls="cone"))
    assert [a for a, _ in ledger.query(agent_ids=[3])] == [3]
    assert sum(len(g) for _, g in ledger.query(class_name="cone")) == 2
    assert sum(len(g) for _, g in ledger.query(since=2.0)) == 2


def test_missing_seq():
    ledger = ObservationLedger()
    for seq in (1, 2, 5):
        ledger.add(obs(6, seq))
    assert ledger.missing_seq(6, 100) == [3, 4]
    assert ledger.missing_seq(6, 999) == []  # unknown session


def test_dict_round_trip():
    o = obs(6, 1)
    assert Observation.from_dict(o.to_dict()) == o


def test_pass_through_resolver():
    o = obs(6, 1)
    assert PassThroughResolver().resolve(o, ObservationLedger()) == o.obs_id


def at(observer, seq, x, y, stamp, cls="chair", object_id=None):
    o = obs(observer, seq, stamp=stamp, cls=cls)
    o.x, o.y = x, y
    if object_id is not None:
        o.object_id = object_id
    return o


def test_gated_resolver_matches_same_class_nearby():
    ledger = ObservationLedger()
    a = at(3, 1, 1.0, 1.0, stamp=1.0)
    ledger.add(a)
    resolver = GatedResolver(radius_m=0.75)
    assert resolver.resolve(at(6, 1, 1.5, 1.0, stamp=2.0), ledger) == a.object_id


def test_gated_resolver_new_object_for_other_class_or_far():
    ledger = ObservationLedger()
    ledger.add(at(3, 1, 1.0, 1.0, stamp=1.0))
    resolver = GatedResolver(radius_m=0.75)
    cone = at(6, 1, 1.0, 1.0, stamp=2.0, cls="cone")
    far = at(6, 2, 5.0, 1.0, stamp=3.0)
    assert resolver.resolve(cone, ledger) == cone.obs_id
    assert resolver.resolve(far, ledger) == far.obs_id


def test_gated_resolver_nearest_wins():
    ledger = ObservationLedger()
    ledger.add(at(3, 1, 0.0, 0.0, stamp=1.0))
    near = at(3, 2, 1.0, 0.0, stamp=2.0)
    ledger.add(near)
    assert GatedResolver(radius_m=1.0).resolve(at(6, 1, 0.7, 0.0, stamp=3.0), ledger) == near.object_id


def test_reconcile_merges_race_to_earliest():
    # Agents 3 and 6 each saw the chair before hearing from the other.
    a = at(3, 1, 1.0, 1.0, stamp=1.0)
    b = at(6, 1, 1.2, 1.0, stamp=2.0)
    for first, second in ((a, b), (b, a)):  # arrival order must not matter
        ledger = ObservationLedger()
        resolver = GatedResolver(radius_m=0.75)
        ledger.add(first)
        assert not ledger.reconcile(first, resolver)
        ledger.add(second)
        assert ledger.reconcile(second, resolver)
        assert ledger.canonical(a.object_id) == ledger.canonical(b.object_id) == a.obs_id
        assert list(ledger.objects()) == [a.obs_id]


def test_canonical_waits_for_origin_observation():
    # b refers to a's object before a itself has arrived; once a arrives, ids still agree.
    a = at(3, 1, 1.0, 1.0, stamp=1.0)
    b = at(6, 1, 1.1, 1.0, stamp=2.0, object_id=a.obs_id)
    ledger = ObservationLedger()
    ledger.add(b)
    assert ledger.canonical(b.object_id) == a.obs_id
    ledger.add(a)
    assert not ledger.reconcile(a, GatedResolver())
    assert ledger.objects() == {a.obs_id: [a, b]}


def test_objects_groups_by_canonical_id():
    ledger = ObservationLedger()
    a = at(3, 1, 1.0, 1.0, stamp=1.0)
    b = at(6, 1, 1.1, 1.0, stamp=2.0, object_id=a.obs_id)
    c = at(6, 2, 9.0, 9.0, stamp=3.0)
    for o in (a, b, c):
        ledger.add(o)
    assert ledger.objects() == {a.obs_id: [a, b], c.obs_id: [c]}


def removal(observer, seq, object_id, stamp, x=1.0, y=1.0, cls="chair", session=100):
    return Removal(make_obs_id(observer, session, seq), object_id, observer, session, seq, stamp, cls, x, y)


def merged_chair():
    """Chair seen by agents 3 and 6 under one object id."""
    ledger = ObservationLedger()
    a = at(3, 1, 1.0, 1.0, stamp=1.0)
    b = at(6, 1, 1.1, 1.0, stamp=2.0, object_id=a.obs_id)
    for o in (a, b):
        ledger.add(o)
    return ledger, a, b


def test_remove_deletes_all_observations_of_object():
    ledger, a, b = merged_chair()
    cone = at(6, 2, 5.0, 5.0, stamp=3.0, cls="cone")
    ledger.add(cone)
    assert ledger.remove(removal(9, 1, a.obs_id, stamp=10.0))
    assert ledger.objects() == {cone.obs_id: [cone]}
    assert ledger.query() == [(6, [cone])]
    assert ledger.agent_ids() == [6]
    assert ledger.is_removed(a.obs_id)


def test_remove_covers_merged_ids():
    ledger = ObservationLedger()
    resolver = GatedResolver()
    a = at(3, 1, 1.0, 1.0, stamp=1.0)
    b = at(6, 1, 1.2, 1.0, stamp=2.0)  # own id, merged by reconcile
    for o in (a, b):
        ledger.add(o)
        ledger.reconcile(o, resolver)
    ledger.remove(removal(9, 1, a.obs_id, stamp=10.0))
    assert ledger.objects() == {}
    assert ledger.is_removed(b.obs_id)


def test_remove_is_idempotent():
    ledger, a, _ = merged_chair()
    r = removal(9, 1, a.obs_id, stamp=10.0)
    assert ledger.remove(r)
    assert not ledger.remove(r)
    assert ledger.removals() == [r]


def test_late_observation_of_removed_object_is_ignored():
    ledger, a, _ = merged_chair()
    ledger.remove(removal(9, 1, a.obs_id, stamp=10.0))
    late = at(7, 1, 1.0, 1.0, stamp=5.0, object_id=a.obs_id)
    assert not ledger.add(late)
    assert ledger.objects() == {}
    assert ledger.missing_seq(7, 100) == []  # seq still counted


def test_removal_before_observations():
    ledger = ObservationLedger()
    a = at(3, 1, 1.0, 1.0, stamp=1.0)
    ledger.remove(removal(9, 1, a.obs_id, stamp=10.0))
    assert not ledger.add(a)
    assert ledger.objects() == {}


def test_absorb_stale_drops_older_nearby_same_class():
    ledger, a, _ = merged_chair()
    ledger.remove(removal(9, 1, a.obs_id, stamp=10.0))
    stale = at(7, 1, 1.2, 1.0, stamp=5.0)  # seen before removal, own new id
    assert ledger.add(stale)
    assert ledger.absorb_stale(stale, radius_m=0.75)
    assert ledger.objects() == {}
    assert not ledger.add(stale)


def test_absorb_stale_keeps_newer_or_other_class_or_far():
    ledger, a, _ = merged_chair()
    ledger.remove(removal(9, 1, a.obs_id, stamp=10.0))
    newer = at(7, 1, 1.0, 1.0, stamp=20.0)
    cone = at(7, 2, 1.0, 1.0, stamp=5.0, cls="cone")
    far = at(7, 3, 5.0, 1.0, stamp=5.0)
    for o in (newer, cone, far):
        ledger.add(o)
        assert not ledger.absorb_stale(o, radius_m=0.75)
    assert len(ledger.objects()) == 3


def test_resighting_after_removal_is_new_object():
    ledger, a, _ = merged_chair()
    ledger.remove(removal(9, 1, a.obs_id, stamp=10.0))
    again = at(6, 2, 1.0, 1.0, stamp=20.0)
    assert GatedResolver().resolve(again, ledger) == again.obs_id


def test_mark_complete_clears_gaps():
    ledger = ObservationLedger()
    for seq in (1, 2, 5, 8):
        ledger.add(obs(6, seq))
    ledger.mark_complete(6, 100, 5)
    assert ledger.missing_seq(6, 100) == [6, 7]
    ledger.mark_complete(6, 100, 3)  # never moves backwards
    assert ledger.missing_seq(6, 100) == [6, 7]


def test_removal_dict_round_trip():
    r = removal(9, 1, "3-100-1", stamp=10.0)
    assert Removal.from_dict(r.to_dict()) == r

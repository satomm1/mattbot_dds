"""Time-decaying blockage belief, per object and per map cell, derived from the observation ledger.

Pure Python + numpy (no ROS) so it can be unit-tested.

Belief: -1 = free, 0 = unknown, +1 = blocked (b = 2 P(blocked) - 1).
  - Object belief: +1 for t1 seconds after the object's latest observation (any agent), then
    linear decay to 0 over t2 seconds, then 0. Removed objects are gone from the ledger.
  - Cell belief: max object belief over footprints covering the cell; -1 if none.
Every agent holding the same ledger computes the same beliefs (given synced clocks).
"""

import math
from dataclasses import dataclass, field
from typing import List

import numpy as np

# Belief phases
HELD = 0  # within t1 of the latest observation: belief 1
DECAYING = 1  # t1 .. t1 + t2: belief falling linearly
UNKNOWN = 2  # past t1 + t2: belief 0


@dataclass
class TrackedObject:
    """One physical object, summarised from all of its ledger observations."""

    object_id: str
    class_name: str
    x: float  # mean position, reference frame
    y: float
    local_x: float  # mean position, this robot's map frame
    local_y: float
    width: float  # max observed width (m)
    first_stamp: float  # earliest observation (Unix wall time, s)
    last_stamp: float  # latest observation
    num_observations: int
    observer_ids: List[int] = field(default_factory=list)  # sorted, unique


@dataclass
class ObjectBeliefState:
    obj: TrackedObject
    belief: float
    phase: int  # HELD / DECAYING / UNKNOWN
    age: float  # now - last_stamp (s)
    time_to_decay: float  # s until decay starts (0 once started)
    time_to_unknown: float  # s until belief reaches 0 (0 once reached)


def object_belief(age_s, t1, t2):
    """Belief that an object last observed age_s seconds ago still blocks its cells."""
    if age_s <= t1:  # includes age < 0 (observer clock ahead of ours)
        return 1.0
    if t2 <= 0 or age_s >= t1 + t2:
        return 0.0
    return 1.0 - (age_s - t1) / t2


def object_state(obj, now, t1, t2):
    """Belief, phase and time-to-transition of one tracked object."""
    age = now - obj.last_stamp
    belief = object_belief(age, t1, t2)
    if age <= t1:
        phase = HELD
    elif belief > 0.0:
        phase = DECAYING
    else:
        phase = UNKNOWN
    return ObjectBeliefState(
        obj=obj,
        belief=belief,
        phase=phase,
        age=age,
        time_to_decay=max(t1 - age, 0.0),
        time_to_unknown=max(t1 + max(t2, 0.0) - age, 0.0),
    )


def objects_from_ledger(agent_entries):
    """Group ledger observations by object_id into TrackedObjects (sorted by object_id).

    agent_entries: iterable of items with .observations (e.g. Ledger.agents), whose
    observations have object_id, class_name, observer_id, x, y, local_x, local_y, width, stamp.
    """
    groups = {}
    for entry in agent_entries:
        for o in entry.observations:
            groups.setdefault(o.object_id, []).append(o)
    objects = []
    for object_id in sorted(groups):
        members = groups[object_id]
        n = len(members)
        latest = max(members, key=lambda o: o.stamp)
        objects.append(
            TrackedObject(
                object_id=object_id,
                class_name=latest.class_name,
                x=sum(o.x for o in members) / n,
                y=sum(o.y for o in members) / n,
                local_x=sum(o.local_x for o in members) / n,
                local_y=sum(o.local_y for o in members) / n,
                width=max(o.width for o in members),
                first_stamp=min(o.stamp for o in members),
                last_stamp=latest.stamp,
                num_observations=n,
                observer_ids=sorted({o.observer_id for o in members}),
            )
        )
    return objects


def belief_grid(states, width, height, resolution, origin_x, origin_y):
    """(height, width) float32 cell belief grid from ObjectBeliefStates."""
    grid = np.full((height, width), -1.0, dtype=np.float32)
    for s in states:
        x, y, half = s.obj.local_x, s.obj.local_y, s.obj.width / 2.0
        # Inclusive cell range; always contains the centre cell, so footprint >= 1 cell.
        i0 = int(math.floor((x - half - origin_x) / resolution))
        i1 = int(math.floor((x + half - origin_x) / resolution))
        j0 = int(math.floor((y - half - origin_y) / resolution))
        j1 = int(math.floor((y + half - origin_y) / resolution))
        i0, i1 = max(i0, 0), min(i1, width - 1)
        j0, j1 = max(j0, 0), min(j1, height - 1)
        if i0 > i1 or j0 > j1:
            continue  # entirely outside the map
        cells = grid[j0:j1 + 1, i0:i1 + 1]
        np.maximum(cells, s.belief, out=cells)
    return grid


def belief_to_occupancy(grid):
    """Belief grid -> OccupancyGrid data (row-major): free 0, unknown 50, blocked 100."""
    return np.rint(50.0 * (grid + 1.0)).astype(np.int8).flatten().tolist()

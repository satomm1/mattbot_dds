"""In-memory ledger of object observations, keyed by observing agent.

Pure Python (no ROS) so it can be unit-tested. Positions are stored in the shared
reference frame so observations from different agents are directly comparable.

Identity model:
  obs_id    -- unique, immutable id of one observation: "{observer_id}-{session}-{seq}".
  object_id -- id of the physical object, assigned by the observer: either a new id (its own
               obs_id) or the canonical id of a matching object already in its ledger.

Two agents can assign different object_ids to the same object (e.g. both see it before
hearing from each other). reconcile() merges such ids; canonical() maps any object_id to
the id of the earliest observation of the merged object, so all agents converge on the
same id once they hold the same observations. Stored observations are never rewritten.

Removal: once an object is observed to be gone, a Removal record deletes all observations of
its (merged) object and leaves a tombstone on each of its object_ids. The ledger therefore only
holds active objects; tombstones stop late or stale copies from resurrecting a removed object.
A later sighting at the same spot becomes a new object.
"""

import math
from collections import OrderedDict
from dataclasses import asdict, dataclass


@dataclass
class Observation:
    obs_id: str
    object_id: str
    observer_id: int  # ROBOT_ID of the agent that made the observation
    session: int  # observer's ledger start time (s); keeps obs_id unique across restarts
    seq: int  # 1, 2, 3, ... per (observer, session); used to detect dropped DDS samples
    stamp: float  # observer's ROS time (s; Unix time: the wall clock on robots, the sim clock in simulation)
    class_name: str
    probability: float
    width: float  # metres
    x: float  # reference frame
    y: float  # reference frame

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


@dataclass
class Removal:
    """An agent observed that an object is gone. Shares the observer's seq counter."""

    rem_id: str  # make_obs_id(observer_id, session, seq)
    object_id: str  # canonical id of the removed object at the remover
    observer_id: int
    session: int
    seq: int
    stamp: float  # observer's ROS time (s; Unix time: the wall clock on robots, the sim clock in simulation)
    class_name: str
    x: float  # last known position of the object (reference frame)
    y: float

    def to_dict(self):
        return asdict(self)

    @classmethod
    def from_dict(cls, d):
        return cls(**d)


def make_obs_id(observer_id, session, seq):
    return "%d-%d-%d" % (observer_id, session, seq)


class ObjectIdResolver:
    """Decides which object_id a new observation belongs to."""

    def resolve(self, obs, ledger):
        raise NotImplementedError


class PassThroughResolver(ObjectIdResolver):
    """Every observation is treated as a new object."""

    def resolve(self, obs, ledger):
        return obs.obs_id


class GatedResolver(ObjectIdResolver):
    """Same class within radius_m (reference frame) of an existing object -> same object.

    Distance is to the object's centroid; the nearest match wins. The object obs already
    belongs to is skipped, so this also works for observations already in the ledger.
    """

    def __init__(self, radius_m=0.75):
        self.radius_m = radius_m

    def resolve(self, obs, ledger):
        own = ledger.canonical(obs.object_id)
        best_id, best_d = obs.obs_id, self.radius_m
        for object_id, members in ledger.objects().items():
            if object_id == own:
                continue
            same = [o for o in members if o.class_name == obs.class_name]
            if not same:
                continue
            cx = sum(o.x for o in same) / len(same)
            cy = sum(o.y for o in same) / len(same)
            d = math.hypot(obs.x - cx, obs.y - cy)
            if d <= best_d:
                best_id, best_d = object_id, d
        return best_id


class ObservationLedger:
    """Observations grouped by observer_id; each group ordered by insertion."""

    def __init__(self):
        self._by_agent = {}  # observer_id -> OrderedDict[obs_id -> Observation]
        self._seqs = {}  # (observer_id, session) -> set of seqs seen
        self._by_obs_id = {}  # obs_id -> Observation
        self._parent = {}  # union-find over object_ids (roots are arbitrary; see canonical)
        self._removals = {}  # rem_id -> Removal
        self._tombstones = {}  # removed object_id -> Removal
        self._complete = {}  # (observer_id, session) -> seq up to which nothing is missing

    def add(self, obs):
        """Insert obs. Returns False if it was already present or its object was removed."""
        if obs.object_id in self._tombstones:
            self._seqs.setdefault((obs.observer_id, obs.session), set()).add(obs.seq)
            return False
        entries = self._by_agent.setdefault(obs.observer_id, OrderedDict())
        if obs.obs_id in entries:
            return False
        entries[obs.obs_id] = obs
        self._by_obs_id[obs.obs_id] = obs
        self._seqs.setdefault((obs.observer_id, obs.session), set()).add(obs.seq)
        self._parent.setdefault(obs.object_id, obs.object_id)
        return True

    # ---------- Object identity ----------

    def _find(self, object_id):
        parent = self._parent
        root = object_id
        while parent.get(root, root) != root:
            root = parent[root]
        while object_id != root:  # path compression
            parent[object_id], object_id = root, parent[object_id]
        return root

    def _id_key(self, object_id):
        """Order object_ids by the stamp of the observation that created them."""
        o = self._by_obs_id.get(object_id)
        return (o.stamp if o is not None else math.inf, object_id)

    def canonical(self, object_id):
        """Id of the earliest-created object_id merged with object_id (itself if unmerged)."""
        root = self._find(object_id)
        members = [oid for oid in self._parent if self._find(oid) == root]
        return min(members, key=self._id_key) if members else object_id

    def merge(self, a, b):
        """Record that object_ids a and b are the same physical object."""
        self._parent.setdefault(a, a)
        self._parent.setdefault(b, b)
        ra, rb = self._find(a), self._find(b)
        if ra != rb:
            self._parent[rb] = ra

    def reconcile(self, obs, resolver):
        """Merge obs's object with a different matching object, if any. Returns True on merge."""
        match = resolver.resolve(obs, self)
        if match == obs.obs_id or self.canonical(match) == self.canonical(obs.object_id):
            return False
        self.merge(match, obs.object_id)
        return True

    def objects(self):
        """{canonical object_id: [Observation, ...]} with each list oldest first."""
        canon = {}
        result = {}
        for o in self._by_obs_id.values():
            if o.object_id not in canon:
                canon[o.object_id] = self.canonical(o.object_id)
            result.setdefault(canon[o.object_id], []).append(o)
        for members in result.values():
            members.sort(key=lambda o: o.stamp)
        return result

    def agent_ids(self):
        return sorted(self._by_agent)

    def observations_of(self, agent_id):
        """All observations by one agent, oldest first."""
        return sorted(self._by_agent.get(agent_id, {}).values(), key=lambda o: o.stamp)

    def query(self, agent_ids=None, class_name="", since=0.0):
        """Return [(agent_id, [Observation, ...]), ...] sorted by agent id, then stamp.

        Empty/None agent_ids means all agents; empty class_name means all classes.
        """
        result = []
        for aid in self.agent_ids():
            if agent_ids and aid not in agent_ids:
                continue
            obs = [
                o
                for o in self.observations_of(aid)
                if o.stamp >= since and (not class_name or o.class_name == class_name)
            ]
            if obs:
                result.append((aid, obs))
        return result

    def missing_seq(self, agent_id, session):
        """Sequence numbers below the highest one seen that never arrived."""
        seen = self._seqs.get((agent_id, session), set())
        if not seen:
            return []
        start = self._complete.get((agent_id, session), 0) + 1
        return sorted(set(range(start, max(seen) + 1)) - seen)

    def mark_complete(self, agent_id, session, max_seq):
        """The observer confirmed we hold everything it still has up to max_seq.

        Needed because an observer cannot resend observations it deleted on removal.
        """
        key = (agent_id, session)
        self._complete[key] = max(self._complete.get(key, 0), max_seq)

    # ---------- Removal ----------

    def remove(self, removal):
        """Apply a removal. Returns False if it was already applied."""
        if removal.rem_id in self._removals:
            return False
        self._removals[removal.rem_id] = removal
        self._seqs.setdefault((removal.observer_id, removal.session), set()).add(removal.seq)
        # The object may be unknown here yet; the tombstone still blocks its later observations.
        self._tombstone_object(removal.object_id, removal)
        return True

    def _tombstone_object(self, object_id, removal):
        """Tombstone every object_id merged with object_id and delete their observations."""
        root = self._find(object_id)
        ids = {oid for oid in self._parent if self._find(oid) == root} | {object_id}
        for oid in ids:
            self._tombstones.setdefault(oid, removal)
            self._parent.pop(oid, None)
        for o in [o for o in self._by_obs_id.values() if o.object_id in ids]:
            del self._by_obs_id[o.obs_id]
            entries = self._by_agent[o.observer_id]
            del entries[o.obs_id]
            if not entries:
                del self._by_agent[o.observer_id]

    def absorb_stale(self, obs, radius_m):
        """Drop obs's object if it is a stale sighting of an already-removed object.

        Catches a peer that saw the object before it was removed (under its own new
        object_id) but whose observation arrived after the removal. Applies only if every
        observation of obs's object predates a nearby same-class removal. Returns True if dropped.
        """
        own = self.canonical(obs.object_id)
        members = self.objects().get(own, [obs])
        newest = max(o.stamp for o in members)
        for removal in self._removals.values():
            if (
                removal.class_name == obs.class_name
                and removal.stamp >= newest
                and math.hypot(obs.x - removal.x, obs.y - removal.y) <= radius_m
            ):
                self._tombstone_object(obs.object_id, removal)
                return True
        return False

    def removals(self, agent_ids=None):
        """Removal records (optionally only those made by agent_ids), oldest first."""
        return sorted(
            (r for r in self._removals.values() if not agent_ids or r.observer_id in agent_ids),
            key=lambda r: r.stamp,
        )

    def is_removed(self, object_id):
        return object_id in self._tombstones

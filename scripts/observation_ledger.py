#!/usr/bin/env python3
"""Keeps a ledger of object observations from this robot and its peers.

Only active objects are kept: observed and not yet observed to be gone.

Ego observations come from /confirmed_objects; ego removals ("this object is gone") come from
/removed_objects (same DetectedObject type, local map frame) or the /ledger/remove service.
Peer events arrive over DDS via data_publisher / data_subscriber / own_data_subscriber,
which relay JSON on these std_msgs/String topics:

  out (-> DDS):  /ledger/observation_for_dds, /ledger/removal_for_dds,
                 /ledger/sync_request_for_dds, /ledger/sync_response_for_dds
  in  (<- DDS):  /ledger/observation_from_agent, /ledger/removal_from_agent,
                 /ledger/sync_request_from_agent, /ledger/sync_response

The ledger is exposed as the latched /ledger topic and the /ledger/get service.
"""

import json
import sys
import threading
import time

import rospy
from std_msgs.msg import Float64MultiArray, Int16MultiArray, String
from mattbot_image_detection.msg import DetectedObject
from mattbot_dds.msg import Ledger, LedgerAgentEntry, LedgerObservation
from mattbot_dds.srv import GetLedger, GetLedgerResponse, RemoveObject, RemoveObjectResponse

from dds_utils import (
    INIT_DISCOVERY_GRACE_S,
    ROS_TOPIC_AGENTS_TO_SUBSCRIBE,
    ROS_TOPIC_TRANSFORMATION_MATRIX,
    DdsLogger,
    GatedResolver,
    Observation,
    ObservationLedger,
    Removal,
    RobotIdError,
    TransformMixin,
    make_obs_id,
    require_robot_id_int,
)

_log = DdsLogger("ledger")

SYNC_PERIOD_S = 3.0  # how often to retry sync requests to peers that have not answered
SYNC_MAX_ATTEMPTS = 5  # per peer
GAP_REQUEST_PERIOD_S = 5.0  # min time between gap-recovery requests for one agent


class ObservationLedgerNode(TransformMixin):
    def __init__(self):
        rospy.init_node("observation_ledger")
        # Times are ROS time: the wall clock on the robots, the sim clock with /use_sim_time (0 until the
        # first /clock, so wait for it before stamping the session)
        while not rospy.is_shutdown() and rospy.get_time() <= 0.0:
            time.sleep(0.05)  # wall clock: ROS time is not running yet
        self.init_transform_state()

        try:
            self.my_id = require_robot_id_int()
        except RobotIdError as exc:
            rospy.logfatal("%s", exc)
            sys.exit(1)

        # Ego observation ids are "<my_id>-<session>-<seq>"; a new session on every start.
        self.session = int(rospy.get_time())
        self.seq = 0
        self.start_time = rospy.get_time()

        self.ledger = ObservationLedger()
        # Same class within this radius (reference frame, m) of a known object -> same object
        self.resolver = GatedResolver(float(rospy.get_param("~match_radius_m", 0.75)))
        self.lock = threading.Lock()  # rospy callbacks run on several threads
        self.dirty = True  # /ledger needs republishing

        # Sync bookkeeping
        self.peers = set()  # current peers from /agents_to_subscribe
        self.synced = set()  # peers that have answered a sync request
        self.sync_attempts = {}  # peer -> requests sent
        self.last_gap_request = {}  # agent -> time of last gap-recovery request

        # Outbound to DDS (JSON strings, forwarded by data_publisher)
        self.obs_pub = rospy.Publisher("/ledger/observation_for_dds", String, queue_size=10)
        self.rem_pub = rospy.Publisher("/ledger/removal_for_dds", String, queue_size=10)
        self.req_pub = rospy.Publisher("/ledger/sync_request_for_dds", String, queue_size=10)
        self.resp_pub = rospy.Publisher("/ledger/sync_response_for_dds", String, queue_size=10)
        # Local view of the ledger
        self.ledger_pub = rospy.Publisher("/ledger", Ledger, queue_size=1, latch=True)

        rospy.Subscriber(ROS_TOPIC_TRANSFORMATION_MATRIX, Float64MultiArray, self.transformation_callback)
        rospy.Subscriber(ROS_TOPIC_AGENTS_TO_SUBSCRIBE, Int16MultiArray, self.agents_callback)
        rospy.Subscriber("/confirmed_objects", DetectedObject, self.ego_callback, queue_size=10)
        rospy.Subscriber("/removed_objects", DetectedObject, self.ego_removal_callback, queue_size=10)
        rospy.Subscriber("/ledger/observation_from_agent", String, self.remote_obs_callback, queue_size=50)
        rospy.Subscriber("/ledger/removal_from_agent", String, self.remote_removal_callback, queue_size=50)
        rospy.Subscriber("/ledger/sync_request_from_agent", String, self.sync_request_callback, queue_size=10)
        rospy.Subscriber("/ledger/sync_response", String, self.sync_response_callback, queue_size=10)

        rospy.Service("/ledger/get", GetLedger, self.get_ledger_srv)
        rospy.Service("/ledger/remove", RemoveObject, self.remove_object_srv)

        rospy.Timer(rospy.Duration(SYNC_PERIOD_S), self.sync_timer)
        rospy.Timer(rospy.Duration(1.0), self.publish_timer)  # republish /ledger at most 1 Hz

    # ---------- Ego observations ----------

    def ego_callback(self, msg):
        """Record a confirmed object seen by this robot and broadcast it."""
        # Store positions in the shared reference frame
        x, y, _ = self.transform_point([msg.pose.position.x, msg.pose.position.y, 0.0])

        with self.lock:
            self.seq += 1
            obs_id = make_obs_id(self.my_id, self.session, self.seq)
            obs = Observation(
                obs_id=obs_id,
                object_id=obs_id,
                observer_id=self.my_id,
                session=self.session,
                seq=self.seq,
                stamp=rospy.get_time(),
                class_name=msg.class_name,
                probability=float(msg.probability),
                width=float(msg.width),
                x=float(x),
                y=float(y),
            )
            obs.object_id = self.resolver.resolve(obs, self.ledger)
            self.ledger.add(obs)
            self.dirty = True

        self.obs_pub.publish(String(data=json.dumps(obs.to_dict())))

    # ---------- Ego removals ----------

    def ego_removal_callback(self, msg):
        """This robot observed that an object is gone: remove the nearest matching active object."""
        x, y, _ = self.transform_point([msg.pose.position.x, msg.pose.position.y, 0.0])
        probe = Observation("", "", self.my_id, self.session, 0, rospy.get_time(), msg.class_name,
                            float(msg.probability), float(msg.width), float(x), float(y))
        with self.lock:
            object_id = self.resolver.resolve(probe, self.ledger)
            removal = self.remove_object(object_id) if object_id else None
        if removal is None:
            _log.debug("no active %s near (%.2f, %.2f) to remove", msg.class_name, x, y)
            return
        self.rem_pub.publish(String(data=json.dumps(removal.to_dict())))

    def remove_object_srv(self, req):
        with self.lock:
            removal = self.remove_object(self.ledger.canonical(req.object_id))
        if removal is None:
            return RemoveObjectResponse(success=False, message="no active object %s" % req.object_id)
        self.rem_pub.publish(String(data=json.dumps(removal.to_dict())))
        return RemoveObjectResponse(success=True, message="removed %s" % removal.object_id)

    def remove_object(self, object_id):
        """Remove an active object (call with self.lock held). Returns the Removal or None."""
        members = self.ledger.objects().get(object_id)
        if not members:
            return None
        self.seq += 1
        removal = Removal(
            rem_id=make_obs_id(self.my_id, self.session, self.seq),
            object_id=object_id,
            observer_id=self.my_id,
            session=self.session,
            seq=self.seq,
            stamp=rospy.get_time(),
            class_name=members[0].class_name,
            x=sum(o.x for o in members) / len(members),
            y=sum(o.y for o in members) / len(members),
        )
        self.ledger.remove(removal)
        self.dirty = True
        _log.info("removed object %s (%s)", object_id, removal.class_name)
        return removal

    # ---------- Peer observations / removals ----------

    def remote_obs_callback(self, msg):
        """Record a live observation broadcast by a peer; request a resync if samples were dropped."""
        obs = Observation.from_dict(json.loads(msg.data))
        with self.lock:
            if self.ledger.add(obs):
                self.add_remote(obs)
            gap = self.check_gap(obs.observer_id, obs.session)
        if gap:
            self.request_gap(obs.observer_id, gap)

    def remote_removal_callback(self, msg):
        """Apply a removal broadcast by a peer."""
        removal = Removal.from_dict(json.loads(msg.data))
        with self.lock:
            if self.ledger.remove(removal):
                self.dirty = True
                _log.info("agent %s removed object %s", removal.observer_id, removal.object_id)
            gap = self.check_gap(removal.observer_id, removal.session)
        if gap:
            self.request_gap(removal.observer_id, gap)

    def check_gap(self, agent_id, session):
        """Missing seqs from agent, if a gap request is due (call with self.lock held)."""
        # DataTopic is KeepLast(1), so a burst can drop samples; ask the observer to resend.
        gap = self.ledger.missing_seq(agent_id, session)
        now = rospy.get_time()
        if not gap or now - self.last_gap_request.get(agent_id, 0.0) <= GAP_REQUEST_PERIOD_S:
            return []
        self.last_gap_request[agent_id] = now
        return gap

    def request_gap(self, agent_id, gap):
        _log.info("missing seq %s from agent %s; requesting resync", gap, agent_id)
        self.send_sync_request(responders=[agent_id], agents=[agent_id])

    def add_remote(self, obs):
        """Bookkeeping after a peer observation was added (call with self.lock held)."""
        self.dirty = True
        # A peer may have seen the object before it was removed, but told us only now.
        if self.ledger.absorb_stale(obs, self.resolver.radius_m):
            _log.info("dropped stale sighting %s of removed object", obs.obs_id)
            return
        # The observer may not have known about a matching object yet; merge the ids.
        if self.ledger.reconcile(obs, self.resolver):
            _log.info(
                "merged object %s (agent %s) into %s",
                obs.object_id, obs.observer_id, self.ledger.canonical(obs.object_id),
            )

    # ---------- Sync (late join / gap recovery) ----------

    def agents_callback(self, msg):
        """Track the current peer set (excludes this robot)."""
        peers = set(msg.data)
        with self.lock:
            # Forget sync state for peers that left so we resync if they rejoin
            for gone in self.peers - peers:
                self.synced.discard(gone)
                self.sync_attempts.pop(gone, None)
            self.peers = peers

    def sync_timer(self, _event):
        """Ask every peer that has not answered yet for its full ledger."""
        if rospy.get_time() - self.start_time < INIT_DISCOVERY_GRACE_S:
            return  # give DDS readers time to discover us
        with self.lock:
            pending = [p for p in self.peers - self.synced if self.sync_attempts.get(p, 0) < SYNC_MAX_ATTEMPTS]
            for p in pending:
                self.sync_attempts[p] = self.sync_attempts.get(p, 0) + 1
        if not pending:
            return
        self.send_sync_request(responders=pending, agents=None)

    def send_sync_request(self, responders, agents):
        """Broadcast a request; only `responders` reply, with observations of `agents` (None = all)."""
        payload = {"requester": self.my_id, "responders": list(responders), "agents": agents}
        self.req_pub.publish(String(data=json.dumps(payload)))

    def sync_request_callback(self, msg):
        """Reply directly to the requester with the active objects we know for the requested agents."""
        req = json.loads(msg.data)
        if self.my_id not in req["responders"]:
            return
        agents = req["agents"]
        with self.lock:
            obs = [o.to_dict() for _, group in self.ledger.query(agent_ids=agents) for o in group]
            # Removals let the requester ignore stale copies of removed objects from other peers.
            removals = [r.to_dict() for r in self.ledger.removals(agent_ids=agents)]
            seq = self.seq
        # data_publisher sends this to DataTopic{target}
        payload = {"target": req["requester"], "responder": self.my_id, "observations": obs, "removals": removals}
        if not agents or self.my_id in agents:
            # We sent everything we still have of our own; deleted (removed) seqs won't come.
            payload["seq_high"] = {str(self.session): seq}
        self.resp_pub.publish(String(data=json.dumps(payload)))
        _log.info("sent %d observations, %d removals to agent %s", len(obs), len(removals), req["requester"])

    def sync_response_callback(self, msg):
        """Merge a peer's sync response into the ledger (duplicates are ignored)."""
        resp = json.loads(msg.data)
        with self.lock:
            # Removals first, so stale observations of removed objects are dropped on arrival.
            for d in resp.get("removals", []):
                if self.ledger.remove(Removal.from_dict(d)):
                    self.dirty = True
            added = 0
            for d in resp["observations"]:
                obs = Observation.from_dict(d)
                if self.ledger.add(obs):
                    added += 1
                    self.add_remote(obs)
            for session, max_seq in resp.get("seq_high", {}).items():
                self.ledger.mark_complete(resp["responder"], int(session), int(max_seq))
            self.synced.add(resp["responder"])
        _log.info("sync from agent %s: %d new observations", resp["responder"], added)

    # ---------- Exposure ----------

    def build_ledger_msg(self, agent_ids=None, class_name="", since=0.0):
        """Convert (a filtered view of) the ledger to a Ledger message."""
        out = Ledger()
        out.header.stamp = rospy.Time.now()
        out.header.frame_id = "map"  # frame of local_x/local_y
        with self.lock:
            groups = self.ledger.query(agent_ids, class_name, since)
            canon = {o.object_id: self.ledger.canonical(o.object_id) for _, g in groups for o in g}
        for agent_id, group in groups:
            entry = LedgerAgentEntry(agent_id=agent_id)
            for o in group:
                canonical = canon[o.object_id]
                local_x, local_y, _ = self.transform_point([o.x, o.y, 0.0], forward=False)
                entry.observations.append(
                    LedgerObservation(
                        obs_id=o.obs_id,
                        object_id=canonical,
                        observer_id=o.observer_id,
                        session=o.session,
                        seq=o.seq,
                        stamp=o.stamp,
                        class_name=o.class_name,
                        probability=o.probability,
                        width=o.width,
                        x=o.x,
                        y=o.y,
                        local_x=local_x,
                        local_y=local_y,
                    )
                )
            out.agents.append(entry)
        return out

    def publish_timer(self, _event):
        """Republish /ledger if anything changed since the last publish."""
        if not self.dirty:
            return
        self.dirty = False
        self.ledger_pub.publish(self.build_ledger_msg())

    def get_ledger_srv(self, req):
        return GetLedgerResponse(ledger=self.build_ledger_msg(list(req.agent_ids), req.class_name, req.since))


if __name__ == "__main__":
    ObservationLedgerNode()
    rospy.spin()

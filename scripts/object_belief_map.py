#!/usr/bin/env python3
"""Publishes time-decaying blockage beliefs built from the observation ledger.

Inputs:  /map (geometry), /ledger (observation_ledger.py)
Outputs: /object_beliefs     (mattbot_dds/ObjectBeliefArray, latched): per-object belief, phase and
                             time to the next transition, least certain first; e.g. for deciding
                             which objects to go and re-check.
         /object_belief_map  (nav_msgs/OccupancyGrid, latched, same geometry as /map): per-cell
                             belief; 0 = free (-1), 50 = unknown (0), 100 = blocked (+1).

See dds_utils/belief.py for the model. Not consumed by navigation yet.
"""

import threading

import rospy
from nav_msgs.msg import OccupancyGrid
from mattbot_dds.msg import Ledger, ObjectBelief, ObjectBeliefArray

from dds_utils import belief_grid, belief_to_occupancy, object_state, objects_from_ledger


class ObjectBeliefMapNode:
    def __init__(self):
        rospy.init_node("object_belief_map")
        self.t1 = float(rospy.get_param("~hold_s", 60.0))  # full belief after an observation
        self.t2 = float(rospy.get_param("~decay_s", 120.0))  # then linear decay to unknown
        rate_hz = float(rospy.get_param("~rate_hz", 1.0))

        self.lock = threading.Lock()
        self.map_info = None
        self.map_header = None
        self.objects = []

        self.beliefs_pub = rospy.Publisher("/object_beliefs", ObjectBeliefArray, queue_size=1, latch=True)
        self.grid_pub = rospy.Publisher("/object_belief_map", OccupancyGrid, queue_size=1, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber("/ledger", Ledger, self.ledger_callback, queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / rate_hz), self.publish_timer)

    def map_callback(self, msg):
        with self.lock:
            self.map_info = msg.info
            self.map_header = msg.header

    def ledger_callback(self, msg):
        objects = objects_from_ledger(msg.agents)
        with self.lock:
            self.objects = objects

    def publish_timer(self, _event):
        with self.lock:
            info, header, objects = self.map_info, self.map_header, self.objects
        # Ledger stamps are observers' ROS time (wall time on the robots, the sim clock in simulation),
        # so decay against ROS time too.
        now = rospy.get_time()
        states = sorted((object_state(o, now, self.t1, self.t2) for o in objects), key=lambda s: s.belief)
        stamp = rospy.Time.now()

        out = ObjectBeliefArray(hold_s=self.t1, decay_s=self.t2)
        out.header.stamp = stamp
        out.header.frame_id = "map"  # frame of local_x/local_y
        for s in states:
            o = s.obj
            out.objects.append(
                ObjectBelief(
                    object_id=o.object_id,
                    class_name=o.class_name,
                    x=o.x,
                    y=o.y,
                    local_x=o.local_x,
                    local_y=o.local_y,
                    width=o.width,
                    first_stamp=o.first_stamp,
                    last_stamp=o.last_stamp,
                    num_observations=o.num_observations,
                    observer_ids=o.observer_ids,
                    belief=s.belief,
                    phase=s.phase,
                    age=s.age,
                    time_to_decay=s.time_to_decay,
                    time_to_unknown=s.time_to_unknown,
                )
            )
        self.beliefs_pub.publish(out)

        if info is None:
            return  # no map geometry yet
        grid = belief_grid(states, info.width, info.height, info.resolution,
                           info.origin.position.x, info.origin.position.y)
        msg = OccupancyGrid(info=info, data=belief_to_occupancy(grid))
        msg.header.frame_id = header.frame_id or "map"
        msg.header.stamp = stamp
        self.grid_pub.publish(msg)


if __name__ == "__main__":
    ObjectBeliefMapNode()
    rospy.spin()

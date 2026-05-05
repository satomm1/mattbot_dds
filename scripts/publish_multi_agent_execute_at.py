#!/usr/bin/env python3
"""Orchestrator helper: publish MultiAgentExecuteAt to the DDS trigger topic (data_publisher forwards fleet-wide)."""

from __future__ import print_function

import argparse
import sys

import rospy
from mattbot_dds.msg import MultiAgentExecuteAt


def _parse_fleet_ids(s):
    out = []
    for part in s.replace(";", ",").split(","):
        part = part.strip()
        if not part:
            continue
        out.append(int(part))
    if not out:
        raise ValueError("No fleet ids parsed from %r" % (s,))
    return out


def main():
    parser = argparse.ArgumentParser(
        description="Publish synchronized execute_at for a multi-agent plan (orchestrator machine)."
    )
    parser.add_argument("--plan_id", required=True, help="Must match MultiRobotGoalPlan / external_goal_multi plan_id")
    parser.add_argument(
        "--delay_sec",
        type=float,
        default=5.0,
        help="execute_at = now + delay_sec (wall ROS time on orchestrator)",
    )
    parser.add_argument(
        "--fleet-ids",
        required=True,
        help='Comma or semicolon-separated robot ids, e.g. "0,1,2"',
    )
    parser.add_argument(
        "--topic",
        default=None,
        help="ROS topic (default: rospy param ~publish_topic or /multi_agent_execute_at_dds)",
    )
    rospy.init_node("publish_multi_agent_execute_at", anonymous=True)
    argv = rospy.myargv(argv=sys.argv)
    args = parser.parse_args(argv[1:])
    topic = args.topic or rospy.get_param("~publish_topic", "/multi_agent_execute_at_dds").strip() or "/multi_agent_execute_at_dds"
    try:
        fleet = _parse_fleet_ids(args.fleet_ids)
    except ValueError as e:
        print("Error:", e, file=sys.stderr)
        return 1

    msg = MultiAgentExecuteAt()
    msg.plan_id = args.plan_id
    msg.fleet_robot_ids = fleet
    msg.execute_at = rospy.Time.now() + rospy.Duration(float(args.delay_sec))

    pub = rospy.Publisher(topic, MultiAgentExecuteAt, queue_size=1, latch=True)
    t0 = rospy.Time.now()
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        if (rospy.Time.now() - t0).to_sec() > 5.0:
            rospy.logwarn("No subscribers on %s after 5s; publishing anyway", topic)
            break
        rospy.sleep(0.05)
    pub.publish(msg)
    rospy.loginfo(
        "Published MultiAgentExecuteAt plan_id=%s fleet=%s execute_at=%s to %s",
        msg.plan_id,
        list(msg.fleet_robot_ids),
        msg.execute_at,
        topic,
    )
    rospy.sleep(0.2)
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)

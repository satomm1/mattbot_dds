# MILP waypoint arrival times for coordinated multi-agent plans (one solve, fleet-wide).
# fleet_robot_ids order matches path order in the simultaneous planner; waypoint_times_flat
# concatenates each agent's time vector in that same order (length sum(waypoint_counts)).
string plan_id
int32 source_agent
int32[] fleet_robot_ids
int32[] waypoint_counts
float64[] waypoint_times_flat

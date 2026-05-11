# Snapshot of a robot's timed polyline for sequential multi-agent avoidance (map frame).
# Waypoint times are relative to execute_at (same convention as MultiAgentTimingSolve segments).
int32 robot_id
string plan_id
time execute_at
bool active
nav_msgs/Path path
float64[] waypoint_times
# DDS fan-out targets; if empty, dds_data_publisher only echoes locally. Otherwise each ID gets a DataTopic write.
int32[] dds_forward_robot_ids

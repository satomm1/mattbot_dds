# Distributed collision-pair detection report for one agent pair.
# robot_i / robot_j are fleet member ids (not path indices). segment_i / segment_j are
# parallel arrays of conflicting segment indices on each agent's planned path.
string plan_id
int32 source_agent
int32 robot_i
int32 robot_j
int32[] segment_i
int32[] segment_j
bool complete
float64 sent_stamp   # rospy time (sec) at publisher before DDS forward

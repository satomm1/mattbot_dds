# Where to stop and which objects to observe there (see mattbot_navigation observation_planner).
uint8 OPPORTUNISTIC=0    # kind: stop is on the planned path
uint8 DETOUR=1           # kind: stop is a viewpoint off the path (leave the path, look, replan)
uint8 REMAINING_PATH=0   # resume: continue the remaining trajectory
uint8 REPLAN=1           # resume: replan to the goal from the stop

uint8 kind
uint8 resume
int32 path_index         # index into the request path: the stop (OPPORTUNISTIC) or where to leave it (DETOUR)
float64 x                # stop position (DETOUR: the viewpoint), local map frame
float64 y
float64 heading          # path heading at the stop (turn back to this)
string[] object_ids      # in turn order
geometry_msgs/Point[] targets  # object positions, local map frame
float32[] beliefs
float32[] values         # per object: importance*(1-belief) (OPPORTUNISTIC) or V in m (DETOUR)
float32 cost_s           # estimated extra time for this stop

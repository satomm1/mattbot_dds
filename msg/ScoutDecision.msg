# Whether this robot should go and scout (decision only; nothing executes it yet).
std_msgs/Header header
int32 robot_id
bool idle                      # /robot_mode has been IDLE for at least ~scout_min_idle_s
float64 idle_s                 # s since the robot became IDLE (0 if not idle)
geometry_msgs/Pose2D start     # robot pose the scouts are evaluated from, local map frame
string cost_model              # name of the scout cost model
bool go                        # some candidate is chosen
ScoutCandidate[] candidates    # every evaluated candidate, best net first

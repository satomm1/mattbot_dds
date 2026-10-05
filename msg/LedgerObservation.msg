# One object observation in the ledger (see dds_utils/ledger.py).
string obs_id          # unique id of this observation: "<observer_id>-<session>-<seq>"
string object_id       # canonical id of the physical object (merged across agents)
int32 observer_id      # ROBOT_ID of the observing agent
uint32 session
uint32 seq
float64 stamp          # Unix wall time (s)
string class_name
float32 probability
float32 width          # metres
float64 x              # shared reference frame
float64 y
float64 local_x        # this robot's map frame
float64 local_y

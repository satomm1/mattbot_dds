# Belief that one tracked object is still in place (see dds_utils/belief.py).
uint8 HELD=0           # within hold_s of the latest observation: belief 1
uint8 DECAYING=1       # belief falling linearly toward 0
uint8 UNKNOWN=2        # belief 0

string object_id       # canonical ledger object id
string class_name
float64 x              # mean observed position, shared reference frame
float64 y
float64 local_x        # mean observed position, this robot's map frame
float64 local_y
float32 width          # max observed width (m)
float64 first_stamp    # earliest observation (Unix wall time, s)
float64 last_stamp     # latest observation, any agent
int32 num_observations
int32[] observer_ids   # agents that have observed it

float32 belief         # 1 = blocked .. 0 = unknown
uint8 phase            # HELD / DECAYING / UNKNOWN
float64 age            # s since last_stamp
float64 time_to_decay  # s until decay starts (0 once started)
float64 time_to_unknown  # s until belief reaches 0 (0 once reached)

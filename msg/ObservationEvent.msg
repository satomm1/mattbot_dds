# Navigator observation progress; the perception check uses [window_start, window_end].
uint8 STARTED=0          # facing the object, observation window opened
uint8 ENDED=1            # window closed normally
uint8 ABORTED=2          # interrupted (stop, new goal, person, localization)

std_msgs/Header header
uint8 event
uint8 kind               # ObservationStop kind
float32 cost_s           # estimated cost of the whole stop
string object_id
float64 target_x         # object position, local map frame
float64 target_y
float64 x                # robot pose while observing, local map frame
float64 y
float64 theta
float32 distance         # robot to object (m)
float64 window_start     # ROS time (s; Unix time: the wall clock on robots, the sim clock in simulation), same clock as ledger stamps
float64 window_end       # 0 until ENDED / ABORTED

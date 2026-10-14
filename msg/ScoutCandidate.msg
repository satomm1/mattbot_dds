# One possible scout trip (see mattbot_navigation observation_planner, navigation_utils/scout.py).
string[] object_ids            # in turn order
geometry_msgs/Point[] targets  # object positions, local map frame
float32[] beliefs
float32[] values_m             # per object: V (m)
float64 x                      # viewpoint, local map frame
float64 y
float64 heading                # arrival heading at the viewpoint
float32 to_view_m              # start -> viewpoint
float32 return_m               # viewpoint -> start
float32 value_m                # V: sum of values_m
float32 cost_m                 # C, from the planner's scout cost model
float32 opportunity_m          # part of C for being away (fixed + per second away); 0 if the model has none
float32 t_away_s               # time the robot is away (s); 0 if the model does not report it
float32 net_m                  # V - C
bool chosen                    # worth sending a scout
string blocked                 # why it may not be chosen whatever its value ("" = no reason)

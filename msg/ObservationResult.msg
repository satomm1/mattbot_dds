# Outcome of one observation window (mattbot_image_detection observation_evaluator.py).
uint8 PRESENT=0          # object seen: re-sighting sent to /confirmed_objects
uint8 ABSENT=1           # object gone: removal sent to /removed_objects
uint8 INCONCLUSIVE=2     # not enough evidence either way; ledger unchanged

std_msgs/Header header
string object_id
string class_name
uint8 outcome
string reason
float64 window_start     # Unix wall time (s)
float64 window_end
int32 frames             # detector frames in the window
int32 present_frames
int32 absent_frames
int32 uncertain_frames
string[] uncertain_reasons   # e.g. "occluded", "depth_hit"
int32[] uncertain_counts
bool depth_used
string[] depth_modes         # per-frame depth modes seen: require / veto / uninformative
float32 mean_depth_valid
float32 mean_depth_hit
float32 mean_depth_through
float32 mean_depth_near      # pixels nearer than the object (something in front of it)
string[] resighted_object_ids  # other known objects seen during the window (re-sightings sent)

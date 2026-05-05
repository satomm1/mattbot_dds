# One robot's external goal with multi-robot planning metadata (published by own_data_subscriber on /external_goal_multi).
geometry_msgs/Pose2D goal
string plan_id
bool coordinated
int32 source_agent
int16 target_agent
# Ordered fleet roster (same order as MultiRobotGoalPlan.goals); agent index k uses fleet_robot_ids[k].
int32[] fleet_robot_ids

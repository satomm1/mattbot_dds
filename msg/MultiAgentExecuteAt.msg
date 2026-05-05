# Synchronized trajectory start for coordinated multi-agent plans (map / ROS time).
# Orchestrator sends to each robot's DataTopic; fleet_robot_ids matches MultiRobotGoalPlan order.
string plan_id
time execute_at
int32[] fleet_robot_ids

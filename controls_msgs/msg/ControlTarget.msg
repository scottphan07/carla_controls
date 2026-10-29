# Planning -> controls: what the car should do next.
# Frame: "map", ROS convention (x forward, y LEFT, yaw in radians, counter-clockwise positive).
# Stamp the header with SIM time (use_sim_time:=true) so controls can detect stale targets.

std_msgs/Header header

float32 desired_speed   # [m/s]
float64 target_x        # desired position [m]
float64 target_y        # [m]
bool has_target_yaw     # false = point the car at (target_x, target_y)
float64 target_yaw      # desired direction [rad], used only if has_target_yaw
bool brake_request      # planning wants the car stopped

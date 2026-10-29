# Controls -> planning / perception: what controls just sent to the car, once per sim tick.
# For the car's position and speed, subscribe to /carla/ego_vehicle/odometry directly.

std_msgs/Header header

float32 throttle        # [0, 1]
float32 steer           # [-1, 1], positive = LEFT (ROS convention; the bridge gets the opposite sign)
float32 brake           # [0, 1]
string stop_reason      # "" = driving normally; otherwise target_speed_zero, emergency_stop, obstacle, missing_data

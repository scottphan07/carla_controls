# CARLA controls and sensors

ROS 2 packages that sit between the team's code and CARLA: **sensors** gets camera and
lidar data out of CARLA, **controls** takes driving targets in and drives the car. CARLA is reached
through the [CARLA ROS bridge](https://github.com/carla-simulator/ros-bridge).
Both are wrappers only: the ROS/CARLA plumbing is done, and the control
algorithms (`speed_control.py`, `steering_control.py`, `stop_behavior.py`) are stubs
marked TODO that return a neutral command, so the car stays still until they're written.
The sensors node passes data straight through; its processing hooks are TODO too.

```
 CARLA ROS bridge ── /carla/ego_vehicle/rgb_front/*, /lidar ──► sensors_node ── /sensors/front_camera/*, /sensors/lidar ──► perception

 planning ──── /planning/target (ControlTarget) ─────────┐
 perception ── /perception/obstacle_distance (Range) ────┤
 anyone ────── /controls/emergency_stop (Bool) ──────────┤
                                                         ▼
 CARLA ROS bridge ── /carla/ego_vehicle/odometry ──► controls_node ── /controls/status ──► planning, perception
        ▲                                                │
        └──── /carla/ego_vehicle/vehicle_control_cmd ◄───┘
```

Inside the node, each tick: stop behavior → speed control + steering control →
clip to valid values → publish.

## Layout

This repo holds three ROS 2 packages. Clone it into your workspace's `src/` folder.

| Path | What it is |
|---|---|
| `controls_msgs/msg/ControlTarget.msg` | Planning → controls message (the contract with planning) |
| `controls_msgs/msg/ControlStatus.msg` | Controls → planning/perception status |
| `carla_controls/carla_controls/controls_node.py` | The ROS endpoints: subscriptions, publishers, conversions |
| `carla_controls/carla_controls/messages.py` | Controls input and output data (no ROS) |
| `carla_controls/carla_controls/stop_behavior.py` | Stop behavior |
| `carla_controls/carla_controls/speed_control.py` | Speed control |
| `carla_controls/carla_controls/steering_control.py` | Steering control |
| `carla_controls/carla_controls/controller.py` | Combines them and keeps the command valid |
| `carla_controls/launch/controls.launch.py` | Starts the node on sim time |
| `carla_controls/launch/sim.launch.py` | Starts bridge + ego vehicle + sensors + controls in one command |
| `carla_sensors/config/objects.json` | Ego vehicle and its sensors (the rig) |
| `carla_sensors/carla_sensors/sensors_node.py` | Republishes camera + lidar on `/sensors/*`, with processing hooks |
| `carla_sensors/launch/sensors.launch.py` | Starts the sensors node on sim time |

In `carla_controls`, only `controls_node.py` imports ROS. The rest is plain Python.

## Build and run

Needs ROS 2 Humble, the CARLA ROS bridge (which provides `carla_msgs`, `carla_ros_bridge` and
`carla_spawn_objects`), and a running CARLA server. On the team's shared sim box all of this is already
installed, and CARLA is always running as the `carla` service.

```bash
cd ~/ros2_ws && colcon build --packages-select controls_msgs carla_sensors carla_controls && source install/setup.bash

# Terminal 1: bridge (synchronous, 0.05 s ticks) + ego vehicle with our sensors + sensors node + controls
ros2 launch carla_controls sim.launch.py            # town:=Town03, sensors:=false
# Sensors only (no controls): ros2 launch carla_controls sim.launch.py controls:=false wait_for_control:=False

# Terminal 2: stand-in for planning (drive toward x=50, y=0 at 5 m/s)
ros2 topic pub -r 20 /planning/target controls_msgs/msg/ControlTarget \
    "{header: {stamp: now}, desired_speed: 5.0, target_x: 50.0, target_y: 0.0}"
```

Pick a target near the car's spawn point. The car spawns at a random point unless you add a
`"spawn_point": {"x": ..., "y": ..., "z": ..., "roll": 0, "pitch": 0, "yaw": ...}` entry to the
`ego_vehicle` object in `carla_sensors/config/objects.json` (ROS coordinates; read a good one off
`ros2 topic echo /carla/ego_vehicle/odometry --once`). `stamp: now` gives a wall-clock time, so the
missing-data check is effectively off for this stand-in; the real planning node should
stamp with sim time. Watch `ros2 topic echo /controls/status`
to see what controls is doing and why it is stopping.

Ctrl-C in terminal 1 stops the bridge and removes the car; CARLA keeps running for the next person.

## Sensor data

The sensor rig is `carla_sensors/config/objects.json`. The bridge publishes each sensor there as
`/carla/ego_vehicle/<id>`, once per sim tick (20 Hz), stamped with sim time. `sensors_node`
republishes the camera and lidar on `/sensors/*`; perception should subscribe to those.

| Topic | Type | Sensor |
|---|---|---|
| `/sensors/front_camera/image`, `.../camera_info` | `sensor_msgs/Image`, `CameraInfo` | Front camera, 800×600, 90° (from `sensors_node`) |
| `/sensors/lidar` | `sensor_msgs/PointCloud2` | 32-channel lidar, 50 m (from `sensors_node`) |
| `/carla/ego_vehicle/rgb_front/image`, `/carla/ego_vehicle/lidar` | as above | Raw bridge topics that `sensors_node` reads |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | Ground-truth pose and velocity (controls uses this) |
| `/carla/ego_vehicle/speedometer` | `std_msgs/Float32` | Speed, m/s |
| `/carla/ego_vehicle/vehicle_control_cmd` | `carla_msgs/CarlaEgoVehicleControl` | **Input**: throttle/steer/brake (from controls) |

Run `ros2 topic list | grep -E 'ego_vehicle|sensors'` to confirm. The chase camera, GNSS, IMU,
collision and lane-invasion sensors were taken out of the rig; add them back to `objects.json`
if you need them (sensor types are listed in the CARLA ros-bridge docs). Ways to use the data:

- **Watch:** in the DCV desktop, `ros2 run rqt_image_view rqt_image_view` or `rviz2`
  (CARLA itself has no window).
- **Timing:** the bridge waits for a control command before each tick, so with controls off
  the sensors stop publishing. Launch with `wait_for_control:=False` to test sensors alone.
- **In a node:** subscribe like `sensors_node.py` does, launch with `use_sim_time: True`, and
  convert images with `cv_bridge` (`CvBridge().imgmsg_to_cv2(msg, "bgr8")`).
- **Record and copy off the box:**
  `ros2 bag record -o run1 /sensors/front_camera/image /sensors/lidar /carla/ego_vehicle/odometry /controls/status`,
  then on your laptop `scp -r carla-sim:~/run1 .`

To change the rig (resolution, mounting, more sensors), edit `carla_sensors/config/objects.json` and
rebuild `carla_sensors`. Keep the `odometry` pseudo-sensor: controls depends on it.

## Running on the shared CARLA server

The team shares one CARLA 0.9.16 + ROS 2 Humble machine on AWS and takes turns running tests on it. Ask an admin for access and the access guide; the server setup is kept separately and is not part of this repo.

## Conventions

- **ROS convention everywhere:** frame `map`, x forward, y to the left, yaw in radians
  (counter-clockwise positive), SI units. The bridge publishes odometry this way.
- **Steer sign:** positive = left inside this project and on `/controls/status`. The
  bridge expects CARLA's sign (positive = right), so `controls_node.py` flips it when publishing.
- **Timing:** the bridge publishes odometry once per sim tick, and the node runs the
  control loop in that callback, so controls runs exactly once per tick. Everything uses
  sim time, so planning must stamp its messages with `use_sim_time:=true`.
- **Missing data:** the car brakes until the first `ControlTarget` arrives. Planning should
  publish every tick (20 Hz); a staleness check is a TODO in `stop_behavior.py`.
- **Obstacles:** `sensor_msgs/Range` with `range: .inf` means nothing ahead.

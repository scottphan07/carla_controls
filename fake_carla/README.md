# fake_carla

A stand-in for the CARLA ROS bridge, so the sensors and controls nodes can be run and tested on a
laptop with no simulator and no GPU. It publishes the same topics, message types, frames and
encodings as the real bridge with our rig, filled from a tiny fake world: a straight road with
lane lines, walls on both sides, and a red parked car ahead (it moves further ahead once you pass it).

| Real bridge topic | Fake |
|---|---|
| `/clock` | sim time, +0.05 s per tick, 20 ticks/s |
| `/carla/ego_vehicle/rgb_front/image`, `camera_info` | 800×600 `bgra8`, fx = fy = 400 |
| `/carla/ego_vehicle/lidar` | ~15.5k points per scan, x/y/z/intensity float32, ground at z = −2.4 |
| `/carla/ego_vehicle/odometry`, `speedometer` | from a simple bicycle model |
| `/tf`, `/tf_static` | `map → ego_vehicle → ego_vehicle/{lidar, rgb_front}` |
| `/carla/ego_vehicle/vehicle_control_cmd` (input) | throttle/brake/steer drive the fake car |

Until a control command arrives, the car cruises straight at `cruise_speed` (5 m/s), so the
sensors have something changing to look at.

## Driving test: `drive_test`

`drive_test` stands in for planning and perception, so `controls_node` can drive the car on its own:
it sets a goal straight ahead of where the car starts, publishes `/planning/target` (stamped with sim
time, which controls requires), asks for speed 0 within 3 m of the goal, and publishes
`/perception/obstacle_distance` from the lidar (gap from our front bumper to the nearest thing in
our lane). It prints progress every second and ends with:

- `PASS: stopped 1.6 m from the goal ...` — controls drove there and stopped
- `STOPPED FOR OBSTACLE ...` — something was in the lane and controls stopped for it (also good)
- `FAIL ...` — timed out, with the last `stop_reason` from `/controls/status`

```bash
ros2 run fake_carla drive_test                        # 30 m ahead at 5 m/s
ros2 run fake_carla drive_test --distance 60          # fake world: runs into the parked car at 40 m
ros2 run fake_carla drive_test --speed 8 --no-lidar   # treat the lane as clear
```

It works the same against the fake bridge and against real CARLA.

## Run it on a Mac or PC (Docker)

Needs [Docker Desktop](https://www.docker.com/products/docker-desktop/). From the repo root:

```bash
./docker/fake_sim.sh                     # terminal 1: fake bridge + sensors_node
./docker/ros.sh ros2 run fake_carla check_sensors --save /repo/out/front_camera.png   # terminal 2
```

`check_sensors` listens for 5 s and prints OK/FAIL for each `/sensors/*` topic (rate, encoding,
size, fields), and saves the camera image to `out/front_camera.png`. More:

```bash
./docker/fake_sim.sh controls:=true wait_for_control:=true cruise_speed:=0.0   # + controls_node
./docker/ros.sh ros2 run fake_carla drive_test         # then drive it
./docker/ros.sh ros2 topic hz /sensors/lidar
./docker/ros.sh bash                                   # shell inside, ROS sourced
```

The first `fake_sim.sh` builds the Docker image (a few minutes). After that it starts in
seconds and rebuilds the repo's packages from your current files each time.

## Run it where ROS 2 is installed (e.g. the sim box)

```bash
colcon build --packages-select controls_msgs carla_sensors carla_controls fake_carla
source install/setup.bash
ros2 launch fake_carla fake_sim.launch.py              # don't run this while CARLA's bridge is up
ros2 run fake_carla check_sensors
```

## Unit tests (no ROS needed)

```bash
cd fake_carla && python3 -m pytest test/
```

## Parameters (`fake_sim.launch.py` arguments)

| Argument | Default | |
|---|---|---|
| `cruise_speed` | 5.0 | m/s with no controls; 0 = stand still |
| `wait_for_control` | false | after the first command, tick only when a new command arrives |
| `render_scale` | 2 | camera ray-casts every 2nd pixel (fast); 1 = sharp, ~4× slower |
| `sensors` | true | start `carla_sensors`' `sensors_node` |
| `controls` | false | start `carla_controls`' `controls_node` (needs `carla_msgs`; Docker has it) |

## Files

| File | What it is |
|---|---|
| `fake_carla/scene.py` | The fake world, camera and lidar ray casting, and car physics (plain numpy) |
| `fake_carla/fake_bridge.py` | ROS node publishing the bridge's topics from the scene |
| `fake_carla/checks.py` | Format checks for `/sensors/*` messages (plain Python) |
| `fake_carla/check_sensors.py` | ROS tool: listen, run the checks, print OK/FAIL, save a snapshot |
| `fake_carla/drive_logic.py` | Goal, stop-at-goal and lidar obstacle distance for `drive_test` (plain Python) |
| `fake_carla/drive_test.py` | ROS tool: stand-in planning + perception, drives the car with controls, PASS/FAIL |
| `launch/fake_sim.launch.py` | Fake bridge + sensors (+ controls) |
| `test/test_scene.py` | Unit tests, including fake data through `carla_sensors`' processing |
| `test/test_drive_logic.py` | Unit tests for `drive_test`'s logic |

It's a test double, not a simulator: no traffic, one straight road, simplified physics. Use the
real CARLA box for anything about driving behavior.

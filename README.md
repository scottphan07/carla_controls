# CARLA controls and sensors

ROS 2 packages that sit between the team's code and CARLA: **sensors** gets camera and
lidar data out of CARLA, **controls** takes driving targets in and drives the car. CARLA is reached
through the [CARLA ROS bridge](https://github.com/carla-simulator/ros-bridge).
Controls has first versions of speed control (P with deadband and throttle rate limit), steering
(heading to the target point) and stop behavior; tuning against CARLA is still to do. The sensors
node converts the camera to rgb8 and can filter the lidar (off by default).

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

## Connecting to the pipeline

Other code talks to this pipeline only through ROS 2 topics; nothing needs to import these packages
or know about CARLA. There are two endpoints:

| Endpoint | You... | Topics |
|---|---|---|
| **1. Sensor data out** | subscribe | `/sensors/front_camera/image`, `/sensors/front_camera/camera_info`, `/sensors/lidar`, `/carla/ego_vehicle/odometry` |
| **2. Driving commands in** | publish | `/planning/target`, `/perception/obstacle_distance`, `/controls/emergency_stop` (and subscribe to `/controls/status` for feedback) |

Rules for both:

- **Use sim time.** Start your node with `use_sim_time: True` (examples below). CARLA's clock runs
  on `/clock`, not the wall clock, and every message here is stamped with it.
- **Same machine, same ROS domain.** Your node runs where the pipeline runs: on the sim box, or in
  the Docker fake sim on a laptop. Don't change `ROS_DOMAIN_ID`.
- **Everything ticks at 20 Hz** (one message per topic per 0.05 s of sim time).
- **ROS axes and SI units:** frame `map`, x forward, y left, yaw counter-clockwise in radians, meters,
  m/s.

### Endpoint 1: sensor data out

| Topic | Type | What you get |
|---|---|---|
| `/sensors/front_camera/image` | `sensor_msgs/Image` | 800×600, `rgb8` (3 bytes per pixel, row by row), frame `ego_vehicle/rgb_front` |
| `/sensors/front_camera/camera_info` | `sensor_msgs/CameraInfo` | Pinhole intrinsics: fx = fy = 400, cx = 400, cy = 300, no distortion |
| `/sensors/lidar` | `sensor_msgs/PointCloud2` | ~7–10k points per scan on CARLA (it drops some by default); fields `x, y, z, intensity` (float32) in meters relative to the lidar, which sits 2.4 m above the road (road ≈ z −2.4), 50 m range; frame `ego_vehicle/lidar` |
| `/carla/ego_vehicle/odometry` | `nav_msgs/Odometry` | Ground-truth pose in `map` and velocity in the car's frame |

Camera and lidar from the same tick carry the same `header.stamp`; match them on it.

**Starting template** for any code that uses the sensor data (perception, data collection,
logging, ...). It calls `process()` once per sim tick with that tick's camera image, lidar scan and
car pose, already converted to numpy arrays and plain numbers. Put your code in `process()`.

Save it as `sensor_listener.py` and run `python3 sensor_listener.py` on the sim box (or in the
Docker fake sim) while the pipeline is running. It needs `message_filters`, which comes with ROS 2;
if the import fails, `sudo apt install ros-humble-message-filters`.

```python
import math

import numpy as np
import rclpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2


# ---- conversions: ROS message -> plain Python / numpy ---------------------------------------
def image_to_numpy(msg: Image) -> np.ndarray:
    """rgb8 image -> H x W x 3 uint8 array (600 x 800 x 3)."""
    return np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)


def cloud_to_numpy(msg: PointCloud2) -> np.ndarray:
    """Lidar scan -> N x 4 float32 array of x, y, z, intensity (meters, relative to the lidar)."""
    return point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z", "intensity"))


def odometry_to_pose(msg: Odometry):
    """Odometry -> (x, y, yaw, speed) in the map frame: meters, radians, m/s."""
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw, math.hypot(v.x, v.y)


def stamp_to_seconds(stamp) -> float:
    return stamp.sec + stamp.nanosec * 1e-9


# ---- the node ---------------------------------------------------------------------------------
class SensorListener(Node):
    """Calls process() once per sim tick with that tick's camera image, lidar scan and car pose."""

    def __init__(self):
        super().__init__("sensor_listener",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        # One subscriber per topic; the synchronizer waits until it has all three from the same
        # tick (same timestamp, within 10 ms) and then calls on_tick with them together.
        subscribers = [
            Subscriber(self, Image, "/sensors/front_camera/image"),
            Subscriber(self, PointCloud2, "/sensors/lidar"),
            Subscriber(self, Odometry, "/carla/ego_vehicle/odometry"),
        ]
        self.sync = ApproximateTimeSynchronizer(subscribers, queue_size=10, slop=0.01)
        self.sync.registerCallback(self.on_tick)

    def on_tick(self, image: Image, cloud: PointCloud2, odom: Odometry):
        self.process(
            t=stamp_to_seconds(image.header.stamp),
            rgb=image_to_numpy(image),
            points=cloud_to_numpy(cloud),
            pose=odometry_to_pose(odom),
        )

    def process(self, t, rgb, points, pose):
        """Your code goes here. Runs 20 times per sim second.

        t       sim time [s]
        rgb     600 x 800 x 3 uint8 camera image
        points  N x 4 float32 lidar points: x forward, y left, z up, intensity (road at z ~ -2.4)
        pose    (x, y, yaw, speed) of the car in the map frame
        """
        x, y, yaw, speed = pose
        self.get_logger().info(
            f"t={t:.2f}s  image {rgb.shape}  lidar {len(points)} pts  "
            f"car at ({x:.1f}, {y:.1f}) heading {math.degrees(yaw):.0f} deg, {speed:.1f} m/s")


def main():
    rclpy.init()
    rclpy.spin(SensorListener())


if __name__ == "__main__":
    main()
```

What each part does:

- **Conversion helpers** (`image_to_numpy`, `cloud_to_numpy`, `odometry_to_pose`) turn ROS messages
  into numpy arrays and plain numbers. They work on their own, so you can reuse them in other nodes.
- **`ApproximateTimeSynchronizer`** waits until it has the camera image, lidar scan and odometry
  from the same tick (timestamps within 10 ms), then calls `on_tick` with all three. You never
  get a camera frame paired with a lidar scan from a different moment.
- **`process()`** is the only part you need to change. The template just logs one line per tick:
  `t=87.05s  image (600, 800, 3)  lidar 8732 pts  car at (152.3, 58.1) heading 90 deg, 4.2 m/s`.

To use fewer or different topics, edit the `subscribers` list and the `on_tick` arguments to match
(same order). If you only need one topic, skip the synchronizer and use a plain
`self.create_subscription(Image, "/sensors/front_camera/image", self.on_image, 10)`.

Check it from the command line:

```bash
ros2 topic hz /sensors/lidar                                      # ~20 Hz of sim time
ros2 topic echo /sensors/front_camera/image --once --no-arr       # 800, 600, rgb8
ros2 run fake_carla check_sensors --min-hz 5 --save ~/front.png   # OK/FAIL for all three + a snapshot
```

**Saving data instead of using it live:** record a bag of the raw topics while the sim runs, then
replay it later without CARLA (`ros2 bag play <bag> --clock`):

```bash
ros2 bag record -o run1 /clock /tf /tf_static /carla/ego_vehicle/odometry \
    /carla/ego_vehicle/rgb_front/image /carla/ego_vehicle/rgb_front/camera_info /carla/ego_vehicle/lidar
```

About 2 GB per minute. Record the raw `/carla/...` topics, not `/sensors/...`, so the recording can
be re-processed through `sensors_node` later.

### Endpoint 2: driving commands in

| Topic | Type | Direction | Who |
|---|---|---|---|
| `/planning/target` | `controls_msgs/ControlTarget` | you publish, 20 Hz | planning |
| `/perception/obstacle_distance` | `sensor_msgs/Range` | you publish, 20 Hz | perception |
| `/controls/emergency_stop` | `std_msgs/Bool` | you publish when needed | anyone |
| `/controls/status` | `controls_msgs/ControlStatus` | you subscribe | anyone |

`ControlTarget` (`controls_msgs/msg/ControlTarget.msg`):

| Field | Meaning |
|---|---|
| `header.stamp` | **sim time** (copy it from the odometry message you're answering) |
| `desired_speed` | m/s; ≤ 0.1 means stop |
| `target_x`, `target_y` | the point to drive toward, in `map` |
| `has_target_yaw`, `target_yaw` | optional heading in radians; `false` = aim at the point |
| `brake_request` | `true` = stop now |

`Range` on `/perception/obstacle_distance`: set `header.stamp` (sim time) and `range` = gap in
meters from **our front bumper** to the nearest thing in our lane, or `math.inf` if the lane is
clear. Controls stops under 5 m.

**Controls only drives when both a target and an obstacle range have arrived, stamped with sim
time, at most 0.5 s old.** Otherwise it brakes and reports `stop_reason: missing_data` on
`/controls/status`. Stop reasons, in priority order: `emergency_stop`, `missing_data`, `obstacle`,
`target_speed_zero`; empty = driving.

**Starting template** for any code that drives the car (planning, scripted tests, a perception
node that also sends targets, ...). It calls `decide()` once per sim tick with the car's pose; you
return a `Decision` (speed, target point, obstacle distance, brake), and the template publishes it
to controls with the right sim-time stamps. Put your code in `decide()`.

Save it as `driver.py` and run `python3 driver.py` on the sim box (or in the Docker fake sim) while
`sim.launch.py` is running with controls on. It needs `controls_msgs`, which is in this repo's
workspace (`source ~/ros2_ws/install/setup.bash`).

```python
import math
from dataclasses import dataclass

import rclpy
from controls_msgs.msg import ControlStatus, ControlTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Range
from std_msgs.msg import Bool


# ---- what you decide each tick ----------------------------------------------------------------
@dataclass
class Decision:
    desired_speed: float                     # [m/s]; <= 0.1 means stop
    target_x: float                          # point to drive toward, map frame [m]
    target_y: float
    obstacle_distance: float = math.inf      # gap from our front bumper to the nearest thing in our lane [m]
    brake: bool = False                      # True = stop now


# ---- conversions: plain Python -> ROS message -------------------------------------------------
def odometry_to_pose(msg: Odometry):
    """Odometry -> (x, y, yaw, speed) in the map frame: meters, radians, m/s."""
    p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
    return p.x, p.y, yaw, math.hypot(v.x, v.y)


def make_target(stamp, d: Decision) -> ControlTarget:
    msg = ControlTarget()
    msg.header.stamp = stamp                 # sim time, or controls ignores it as stale
    msg.header.frame_id = "map"
    msg.desired_speed = float(d.desired_speed)
    msg.target_x, msg.target_y = float(d.target_x), float(d.target_y)
    msg.has_target_yaw = False
    msg.brake_request = d.brake
    return msg


def make_range(stamp, distance: float) -> Range:
    msg = Range()
    msg.header.stamp = stamp
    msg.header.frame_id = "ego_vehicle"
    msg.range = float(distance)
    return msg


# ---- the node ---------------------------------------------------------------------------------
class Driver(Node):
    """Calls decide() once per sim tick and sends the result to controls_node."""

    def __init__(self):
        super().__init__("driver",
                         parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.target_pub = self.create_publisher(ControlTarget, "/planning/target", 10)
        self.range_pub = self.create_publisher(Range, "/perception/obstacle_distance", 10)
        self.estop_pub = self.create_publisher(Bool, "/controls/emergency_stop", 10)
        self.create_subscription(Odometry, "/carla/ego_vehicle/odometry", self.on_odometry, 10)
        self.create_subscription(ControlStatus, "/controls/status", self.on_status, 10)
        self.status = None                   # latest ControlStatus from controls_node
        self.goal = None

    def on_status(self, msg: ControlStatus):
        self.status = msg

    def on_odometry(self, odom: Odometry):  # once per sim tick
        t = odom.header.stamp.sec + odom.header.stamp.nanosec * 1e-9
        d = self.decide(t, odometry_to_pose(odom))
        # Reuse the odometry timestamp: it's this tick's sim time.
        self.target_pub.publish(make_target(odom.header.stamp, d))
        self.range_pub.publish(make_range(odom.header.stamp, d.obstacle_distance))

    def emergency_stop(self, on: bool = True):
        self.estop_pub.publish(Bool(data=on))

    def decide(self, t, pose) -> Decision:
        """Your code goes here. Runs 20 times per sim second.

        t      sim time [s]
        pose   (x, y, yaw, speed) of the car in the map frame
        self.status.stop_reason tells you why controls is stopping ("" = driving).

        Example: drive to a point 20 m ahead of where the car started, then stop there.
        """
        x, y, yaw, speed = pose
        if self.goal is None:
            self.goal = (x + 20.0 * math.cos(yaw), y + 20.0 * math.sin(yaw))
        gx, gy = self.goal
        arrived = math.hypot(gx - x, gy - y) < 3.0
        reason = self.status.stop_reason if self.status else "no status yet"
        self.get_logger().info(f"t={t:.2f}s  speed {speed:.1f} m/s  to goal "
                               f"{math.hypot(gx - x, gy - y):.1f} m  controls: {reason or 'driving'}",
                               throttle_duration_sec=1.0)
        return Decision(desired_speed=0.0 if arrived else 5.0, target_x=gx, target_y=gy,
                        brake=arrived)


def main():
    rclpy.init()
    rclpy.spin(Driver())


if __name__ == "__main__":
    main()
```

What each part does:

- **`Decision`** is everything controls needs from you each tick: `desired_speed`, the point to
  drive toward, `obstacle_distance` (leave it at `math.inf` until you have perception; a real
  perception node would fill it from the lidar), and `brake`.
- **`make_target` / `make_range`** turn a `Decision` into the two ROS messages. Both are stamped with
  the odometry message's timestamp, which is this tick's sim time, so controls never treats them as
  stale.
- **`decide()`** is the only part you need to change. The example drives to a point 20 m ahead of
  where the car started and stops there, logging once a second:
  `t=91.00s  speed 3.1 m/s  to goal 11.6 m  controls: driving`.
- **`self.status`** is controls' latest `ControlStatus`; `self.status.stop_reason` says why it's
  braking (`missing_data`, `obstacle`, `target_speed_zero`, `emergency_stop`; empty = driving).
- **`self.emergency_stop(True)`** stops the car from your code; `self.emergency_stop(False)` releases it.

If your node stops publishing (Ctrl-C, crash), controls brakes within 0.5 s. For a ready-made test
that also uses the lidar for obstacles and prints PASS/FAIL, run `ros2 run fake_carla drive_test`
(see [fake_carla/README.md](fake_carla/README.md)).

Check it from the command line:

```bash
ros2 topic echo /controls/status                     # throttle / brake / steer and stop_reason, every tick
ros2 topic info /planning/target -v                  # your node as publisher, "controls" as subscriber
ros2 topic pub --once /controls/emergency_stop std_msgs/msg/Bool "{data: true}"    # stop the car
ros2 topic pub --once /controls/emergency_stop std_msgs/msg/Bool "{data: false}"   # release
```

Plain `ros2 topic pub` on `/planning/target` doesn't work: it stamps with wall-clock time, so
controls treats every target as stale.

### Using it from your own repo

1. **Package dependencies:** reading sensors needs only standard packages (`rclpy`, `sensor_msgs`,
   `sensor_msgs_py`, `nav_msgs`). Publishing targets also needs `<depend>controls_msgs</depend>` in
   your `package.xml`. A plain Python script with no package works too.
2. **On the sim box,** build your code as its own workspace on top of this one (an overlay), so you
   never edit the shared one:
   ```bash
   mkdir -p ~/ws_<name>/src                                    # once
   # from your laptop: rsync -av --delete --exclude .git ./ carla-sim:~/ws_<name>/src/<your_repo>/
   cd ~/ws_<name> && colcon build && source install/setup.bash   # ~/ros2_ws is already sourced
   ```
3. **Start the pipeline** (one person at a time; `ros2 node list | grep carla_ros_bridge` must be
   empty first):
   ```bash
   ros2 launch carla_controls sim.launch.py                                          # driving
   ros2 launch carla_controls sim.launch.py controls:=false wait_for_control:=False  # sensors only
   ```
4. **Run your node** in a second terminal: `ros2 run <your_pkg> <your_node>` or `python3 my_node.py`.
5. **Without the sim box:** the same topics come from the fake bridge in Docker on a laptop (see
   [Testing without CARLA](#testing-without-carla)), so the same code runs against both.

Keep your files in `~/ws_<name>` and recordings in `~/bags/<name>`; don't change `~/ros2_ws` or
`~/carla_ros_ws`.

## Layout

This repo holds three ROS 2 packages, plus `fake_carla` for testing. Clone it into your workspace's `src/` folder.

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
| `carla_sensors/carla_sensors/sensors_node.py` | Republishes camera + lidar on `/sensors/*`; warns when a sensor goes quiet |
| `carla_sensors/carla_sensors/processing.py` | Camera conversion, lidar filter, health watchdog (plain numpy, no ROS) |
| `carla_sensors/config/sensors_params.yaml` | Sensors node settings: camera encoding, lidar filter, timeout |
| `carla_sensors/launch/sensors.launch.py` | Starts the sensors node on sim time |
| `carla_sensors/test/test_processing.py` | Unit tests: `cd carla_sensors && python3 -m pytest test/` |
| `carla_controls/test/` | Controller unit tests: `cd carla_controls && python3 -m pytest test/` |
| `fake_carla/` | Fake CARLA bridge, `check_sensors` and `drive_test` tools, for testing with or without CARLA ([README](fake_carla/README.md)) |
| `docker/` | Runs the fake bridge + our nodes in Docker on a laptop (`./docker/fake_sim.sh`) |

In `carla_controls`, only `controls_node.py` imports ROS. The rest is plain Python.

## Build and run

Needs ROS 2 Humble, the CARLA ROS bridge (which provides `carla_msgs`, `carla_ros_bridge` and
`carla_spawn_objects`), and a running CARLA server. On the team's shared sim box all of this is already
installed, and CARLA is always running as the `carla` service.

```bash
cd ~/ros2_ws && colcon build --packages-select controls_msgs carla_sensors carla_controls fake_carla && source install/setup.bash

# Terminal 1: bridge (synchronous, 0.05 s ticks) + ego vehicle with our sensors + sensors node + controls
ros2 launch carla_controls sim.launch.py            # town:=Town03, sensors:=false
# Sensors only (no controls): ros2 launch carla_controls sim.launch.py controls:=false wait_for_control:=False

# Terminal 2: stand-in for planning + perception: drive 30 m straight ahead, stop, print PASS/FAIL
ros2 run fake_carla drive_test                      # --distance 50 --speed 8 --no-lidar
```

Controls only drives when it gets a planning target **and** an obstacle distance, both stamped with
sim time and less than 0.5 s old, so a plain `ros2 topic pub` (wall-clock stamps, no perception)
leaves the car braking with `stop_reason: missing_data`. `drive_test` publishes both correctly
(see [fake_carla/README.md](fake_carla/README.md)). Watch `ros2 topic echo /controls/status` to see
what controls is doing and why it is stopping.

The car spawns at a random point unless you add a
`"spawn_point": {"x": ..., "y": ..., "z": ..., "roll": 0, "pitch": 0, "yaw": ...}` entry to the
`ego_vehicle` object in `carla_sensors/config/objects.json` (ROS coordinates; read a good one off
`ros2 topic echo /carla/ego_vehicle/odometry --once`). Pick one with straight road ahead for
`drive_test`.

Ctrl-C in terminal 1 stops the bridge and removes the car; CARLA keeps running for the next person.

## Testing without CARLA

`fake_carla` publishes the same topics as the CARLA ROS bridge from a tiny fake world (road, walls,
a parked car), so you can run the sensors and controls nodes on a laptop. With Docker Desktop:

```bash
./docker/fake_sim.sh                     # terminal 1: fake bridge + sensors_node (add controls:=true for controls)
./docker/ros.sh ros2 run fake_carla check_sensors --save /repo/out/front_camera.png   # terminal 2
```

`check_sensors` prints OK/FAIL for each `/sensors/*` topic and saves a camera snapshot to `out/`.
To test driving: `./docker/fake_sim.sh controls:=true wait_for_control:=true cruise_speed:=0.0`, then
`./docker/ros.sh ros2 run fake_carla drive_test`.
Details in [fake_carla/README.md](fake_carla/README.md).

## Sensor data

The sensor rig is `carla_sensors/config/objects.json`. The bridge publishes each sensor there as
`/carla/ego_vehicle/<id>`, once per sim tick (20 Hz), stamped with sim time. `sensors_node`
republishes the camera and lidar on `/sensors/*`; perception should subscribe to those.

| Topic | Type | Sensor |
|---|---|---|
| `/sensors/front_camera/image`, `.../camera_info` | `sensor_msgs/Image`, `CameraInfo` | Front camera, 800×600, 90°, `rgb8` (from `sensors_node`) |
| `/sensors/lidar` | `sensor_msgs/PointCloud2` | 32-channel lidar, 50 m, x/y/z/intensity, filtered per `sensors_params.yaml` (from `sensors_node`) |
| `/carla/ego_vehicle/rgb_front/image`, `/carla/ego_vehicle/lidar` | as above | Raw bridge topics that `sensors_node` reads (image is `bgra8`) |
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
  convert images with `cv_bridge` (`CvBridge().imgmsg_to_cv2(msg, "rgb8")`) or
  `np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.width, 3)`.
- **Record and copy off the box:** see [saving data](#endpoint-1-sensor-data-out) above, then on
  your laptop `scp -r carla-sim:~/run1 .`

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
- **Missing data:** the car brakes unless it has a `ControlTarget` and an obstacle `Range`, each
  stamped with sim time and at most 0.5 s old. Planning and perception should publish every tick (20 Hz).
- **Obstacles:** `sensor_msgs/Range` on `/perception/obstacle_distance`: gap in meters from our
  front bumper to the nearest thing in our lane; `range: .inf` means nothing ahead. Controls stops
  under 5 m.

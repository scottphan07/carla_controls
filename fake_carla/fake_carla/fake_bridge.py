"""ROS 2 node that pretends to be the CARLA ROS bridge, so sensors_node and controls_node can
run with no simulator. It publishes the same topics, types, frames and encodings as the real
bridge with our rig (carla_sensors/config/objects.json), filled from the fake world in scene.py.

Outputs (same names as the real bridge)
  /clock                                    rosgraph_msgs/Clock      sim time, +0.05 s per tick
  /carla/ego_vehicle/rgb_front/image        sensor_msgs/Image        800x600 bgra8
  /carla/ego_vehicle/rgb_front/camera_info  sensor_msgs/CameraInfo
  /carla/ego_vehicle/lidar                  sensor_msgs/PointCloud2  x, y, z, intensity (float32)
  /carla/ego_vehicle/odometry               nav_msgs/Odometry
  /carla/ego_vehicle/speedometer            std_msgs/Float32
  /tf, /tf_static                           map -> ego_vehicle -> ego_vehicle/{lidar, rgb_front}

Input (only if carla_msgs is installed)
  /carla/ego_vehicle/vehicle_control_cmd    carla_msgs/CarlaEgoVehicleControl

Driving: until a control command arrives, the car cruises straight at `cruise_speed`. Once
commands arrive, a simple bicycle model follows them (throttle/brake/steer, CARLA's steer sign).

Parameters
  cruise_speed       [m/s] speed with no controls (default 5.0; 0 = stand still)
  wait_for_control   like the real bridge's synchronous_mode_wait_for_vehicle_control_command:
                     after the first command, only tick once a new command has arrived (default False)
  rate               [Hz, wall clock] tick rate (default 20; sim time always advances 0.05 s per tick)
  lidar_noise        [m] range noise stddev (default 0.0, like our rig)
  render_scale       ray-cast every Nth camera pixel (default 2; 1 = sharp but ~4x slower)
"""

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Float32, Header
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .scene import (CAMERA_H, CAMERA_POS, CAMERA_W, LIDAR_POS, FakeScene, camera_K,
                    yaw_to_quaternion)

try:  # optional: lets controls_node drive the fake car
    from carla_msgs.msg import CarlaEgoVehicleControl
except ImportError:
    CarlaEgoVehicleControl = None

DT = 0.05  # [s] sim time per tick, same as the real bridge's fixed_delta_seconds
BRIDGE = "/carla/ego_vehicle"
LIDAR_FIELDS = [
    PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
    for i, n in enumerate(("x", "y", "z", "intensity"))
]
OPTICAL = (-0.5, 0.5, -0.5, 0.5)  # x, y, z, w: body axes (x fwd, y left, z up) -> optical (z fwd, x right, y down)


class FakeBridge(Node):
    def __init__(self):
        super().__init__("fake_carla_bridge")
        self.cruise_speed = self.declare_parameter("cruise_speed", 5.0).value
        self.wait_for_control = self.declare_parameter("wait_for_control", False).value
        rate = self.declare_parameter("rate", 20.0).value
        self.lidar_noise = self.declare_parameter("lidar_noise", 0.0).value
        self.scene = FakeScene(render_scale=self.declare_parameter("render_scale", 2).value)
        self.rng = np.random.default_rng(0)

        self.control = None          # latest command, or None = cruise
        self.fresh_control = False   # a command arrived since the last tick

        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.image_pub = self.create_publisher(Image, f"{BRIDGE}/rgb_front/image", 10)
        self.info_pub = self.create_publisher(CameraInfo, f"{BRIDGE}/rgb_front/camera_info", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, f"{BRIDGE}/lidar", 10)
        self.odom_pub = self.create_publisher(Odometry, f"{BRIDGE}/odometry", 10)
        self.speed_pub = self.create_publisher(Float32, f"{BRIDGE}/speedometer", 10)
        self.tf = TransformBroadcaster(self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.static_tf.sendTransform([
            self._transform("ego_vehicle", "ego_vehicle/lidar", LIDAR_POS, (0.0, 0.0, 0.0, 1.0), Time()),
            self._transform("ego_vehicle", "ego_vehicle/rgb_front", CAMERA_POS, OPTICAL, Time()),
        ])

        if CarlaEgoVehicleControl is not None:
            self.create_subscription(
                CarlaEgoVehicleControl, f"{BRIDGE}/vehicle_control_cmd", self.on_control, 10)
            controls = "listening on vehicle_control_cmd"
        else:
            controls = "carla_msgs not installed, so cruising only"

        self.create_timer(1.0 / rate, self.tick)
        self.get_logger().info(
            f"fake CARLA bridge at {rate:.0f} Hz; {controls}; cruise_speed={self.cruise_speed} m/s")

    def on_control(self, msg):
        if self.control is None:
            self.get_logger().info("got first vehicle_control_cmd; following controls from now on")
        self.control = msg
        self.fresh_control = True

    # ------------------------------------------------ one sim tick
    def tick(self):
        if self.wait_for_control and self.control is not None and not self.fresh_control:
            return  # synchronous mode: wait for controls to answer the last tick
        self.fresh_control = False

        c = self.control
        if c is None:
            self.scene.cruise(DT, self.cruise_speed)
        else:
            self.scene.step(DT, c.throttle, c.steer, c.brake, c.reverse)

        stamp = Time(nanoseconds=int(round(self.scene.time * 1e9))).to_msg()
        self.clock_pub.publish(Clock(clock=stamp))
        self.publish_vehicle(stamp)
        self.publish_camera(stamp)
        self.publish_lidar(stamp)

    def publish_vehicle(self, stamp):
        car = self.scene.car
        q = yaw_to_quaternion(car.yaw)
        odom = Odometry(header=Header(stamp=stamp, frame_id="map"), child_frame_id="ego_vehicle")
        odom.pose.pose.position.x, odom.pose.pose.position.y = car.x, car.y
        (odom.pose.pose.orientation.x, odom.pose.pose.orientation.y,
         odom.pose.pose.orientation.z, odom.pose.pose.orientation.w) = q
        odom.twist.twist.linear.x = car.speed       # in the car's frame, like the bridge
        odom.twist.twist.angular.z = car.yaw_rate
        self.odom_pub.publish(odom)
        self.speed_pub.publish(Float32(data=float(car.speed)))
        self.tf.sendTransform(self._transform("map", "ego_vehicle", (car.x, car.y, 0.0), q, stamp))

    def publish_camera(self, stamp):
        bgra = self.scene.render_camera()
        header = Header(stamp=stamp, frame_id="ego_vehicle/rgb_front")
        self.image_pub.publish(Image(
            header=header, height=CAMERA_H, width=CAMERA_W, encoding="bgra8",
            is_bigendian=0, step=CAMERA_W * 4, data=bgra.tobytes()))
        K = camera_K()
        info = CameraInfo(header=header, height=CAMERA_H, width=CAMERA_W, distortion_model="plumb_bob")
        info.d = [0.0] * 5
        info.k = K.flatten().tolist()
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [K[0, 0], 0.0, K[0, 2], 0.0, 0.0, K[1, 1], K[1, 2], 0.0, 0.0, 0.0, 1.0, 0.0]
        self.info_pub.publish(info)

    def publish_lidar(self, stamp):
        pts = self.scene.lidar_points(self.rng, self.lidar_noise)
        self.lidar_pub.publish(PointCloud2(
            header=Header(stamp=stamp, frame_id="ego_vehicle/lidar"),
            height=1, width=len(pts), fields=LIDAR_FIELDS, is_bigendian=False,
            point_step=16, row_step=16 * len(pts), is_dense=True, data=pts.tobytes()))

    @staticmethod
    def _transform(parent, child, xyz, q, stamp):
        t = TransformStamped()
        t.header.stamp = stamp if not isinstance(stamp, Time) else stamp.to_msg()
        t.header.frame_id, t.child_frame_id = parent, child
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in xyz)
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = (float(v) for v in q)
        return t


def main():
    rclpy.init()
    node = FakeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

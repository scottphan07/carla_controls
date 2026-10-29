"""ROS 2 node: takes the raw sensor topics from the CARLA ROS bridge, processes them, and
republishes them under /sensors/*, so perception code never has to know it's talking to CARLA.

The bridge itself does the "pull data out of CARLA" part: it reads config/objects.json and
publishes every sensor there as /carla/ego_vehicle/<id>/... once per sim tick (20 Hz).

Inputs (subscriptions, from the CARLA ROS bridge)
  /carla/ego_vehicle/rgb_front/image        sensor_msgs/Image        front camera, 800x600, bgra8
  /carla/ego_vehicle/rgb_front/camera_info  sensor_msgs/CameraInfo   front camera intrinsics
  /carla/ego_vehicle/lidar                  sensor_msgs/PointCloud2  32-channel lidar, 50 m

Outputs (publishers, for perception and anyone else)
  /sensors/front_camera/image               sensor_msgs/Image        rgb8 (or bgra8 if passthrough)
  /sensors/front_camera/camera_info         sensor_msgs/CameraInfo   unchanged
  /sensors/lidar                            sensor_msgs/PointCloud2  x, y, z, intensity; filtered

Parameters (config/sensors_params.yaml)
  camera_encoding      "rgb8" (default) or "passthrough" (bridge's bgra8, no copy)
  lidar_min_range, lidar_max_range, lidar_min_z, lidar_max_z   see processing.LidarFilter
  sensor_timeout       [s, wall time] warn when a sensor has been silent this long

The processing itself lives in processing.py (plain numpy, unit-tested in test/).

TODO:
- Resize/crop the image if perception wants something smaller (update camera_info to match)
- Publish health on a topic (not just log) so other nodes can react
"""
import time

import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2

from .processing import LidarFilter, Watchdog, bgra_to_rgb, filter_points

BRIDGE = "/carla/ego_vehicle"  # role_name of the ego vehicle in the bridge launch
LIDAR_FIELDS = ("x", "y", "z", "intensity")


class SensorsNode(Node):
    def __init__(self):
        super().__init__("sensors")

        self.camera_encoding = self.declare_parameter("camera_encoding", "rgb8").value
        self.lidar_filter = LidarFilter(
            min_range=self.declare_parameter("lidar_min_range", 0.0).value,
            max_range=self.declare_parameter("lidar_max_range", 100.0).value,
            min_z=self.declare_parameter("lidar_min_z", -100.0).value,
            max_z=self.declare_parameter("lidar_max_z", 100.0).value,
        )
        self.watchdog = Watchdog(timeout=self.declare_parameter("sensor_timeout", 2.0).value)

        # Raw topics from the bridge
        self.create_subscription(Image, f"{BRIDGE}/rgb_front/image", self.on_image, 10)
        self.create_subscription(CameraInfo, f"{BRIDGE}/rgb_front/camera_info", self.on_camera_info, 10)
        self.create_subscription(PointCloud2, f"{BRIDGE}/lidar", self.on_lidar, 10)

        # Our topics
        self.image_pub = self.create_publisher(Image, "/sensors/front_camera/image", 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, "/sensors/front_camera/camera_info", 10)
        self.lidar_pub = self.create_publisher(PointCloud2, "/sensors/lidar", 10)

        # Health check on the wall clock: sim time stops when the bridge stops, so a sim-time
        # timer would never fire in exactly the case we want to catch.
        self.create_timer(0.5, self.check_health, clock=Clock(clock_type=ClockType.STEADY_TIME))

        self.get_logger().info(
            f"camera_encoding={self.camera_encoding}, lidar filter={self.lidar_filter}")

    # ------------------------------------------------ callbacks: unpack -> process -> pack
    def on_image(self, msg: Image):
        self._seen("front_camera")
        if self.camera_encoding == "passthrough":
            self.image_pub.publish(msg)
            return
        if msg.encoding != "bgra8":
            self.get_logger().warn(f"expected bgra8 from the bridge, got {msg.encoding}; passing through",
                                   throttle_duration_sec=10.0)
            self.image_pub.publish(msg)
            return
        rgb = bgra_to_rgb(msg.data, msg.height, msg.width, msg.step)
        self.image_pub.publish(Image(
            header=msg.header,
            height=msg.height,
            width=msg.width,
            encoding="rgb8",
            is_bigendian=0,
            step=msg.width * 3,
            data=rgb.tobytes(),
        ))

    def on_camera_info(self, msg: CameraInfo):
        self.camera_info_pub.publish(msg)  # TODO: update if the image is ever resized/cropped

    def on_lidar(self, msg: PointCloud2):
        self._seen("lidar")
        if self.lidar_filter.is_pass_through():
            self.lidar_pub.publish(msg)
            return
        points = point_cloud2.read_points_numpy(msg, field_names=LIDAR_FIELDS).astype(np.float32)
        kept = filter_points(points, self.lidar_filter)
        fields = [f for f in msg.fields if f.name in LIDAR_FIELDS]
        self.lidar_pub.publish(point_cloud2.create_cloud(msg.header, fields, kept))

    # ------------------------------------------------ health
    def _seen(self, name: str):
        if self.watchdog.seen(name, time.monotonic()):
            self.get_logger().info(f"{name} is publishing again")

    def check_health(self):
        for name, age in self.watchdog.check(time.monotonic()):
            self.get_logger().warn(
                f"no {name} data for {age:.1f} s (bridge stopped, or the sim is waiting for a "
                f"control command; launch with wait_for_control:=False to run sensors alone)")


def main():
    rclpy.init()
    node = SensorsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

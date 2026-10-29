"""Listens to the sensors node's outputs for a few seconds and checks them.

    ros2 run fake_carla check_sensors                      # 5 s, expects rgb8 at >= 15 Hz
    ros2 run fake_carla check_sensors --duration 10 --save /tmp/front_camera.png

Works against the fake bridge or the real one. Exits 0 if everything looks right, 1 if not,
so it can be used in scripts.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2

from .checks import check_camera_info, check_image, check_lidar, check_rate, write_png

TOPICS = {
    "/sensors/front_camera/image": Image,
    "/sensors/front_camera/camera_info": CameraInfo,
    "/sensors/lidar": PointCloud2,
}


class Checker(Node):
    def __init__(self):
        super().__init__("check_sensors")
        self.count = {t: 0 for t in TOPICS}
        self.last = {}
        for topic, msg_type in TOPICS.items():
            self.create_subscription(msg_type, topic, lambda m, t=topic: self._got(t, m), 10)

    def _got(self, topic, msg):
        self.count[topic] += 1
        self.last[topic] = msg


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=5.0, help="seconds to listen (wall clock)")
    parser.add_argument("--min-hz", type=float, default=15.0, help="slowest acceptable rate per topic")
    parser.add_argument("--encoding", default="rgb8", help="expected image encoding")
    parser.add_argument("--save", help="save the last camera image here as a PNG")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = Checker()
    print(f"listening for {args.duration:.0f} s ...")
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
    elapsed = time.monotonic() - start

    checks = {
        "/sensors/front_camera/image": lambda m: check_image(m, args.encoding),
        "/sensors/front_camera/camera_info": check_camera_info,
        "/sensors/lidar": check_lidar,
    }
    failed = False
    for topic in TOPICS:
        msg = node.last.get(topic)
        problems = check_rate(node.count[topic], elapsed, args.min_hz)
        problems += checks[topic](msg) if msg is not None else ["no messages received"]
        detail = ""
        if msg is not None and topic.endswith("lidar"):
            detail = f", {msg.width * msg.height} points"
        elif msg is not None and topic.endswith("image"):
            detail = f", {msg.width}x{msg.height} {msg.encoding}"
        status = "OK  " if not problems else "FAIL"
        print(f"{status} {topic}: {node.count[topic] / elapsed:.1f} Hz{detail}")
        for p in problems:
            print(f"       - {p}")
        failed |= bool(problems)

    image = node.last.get("/sensors/front_camera/image")
    if args.save and image is not None and image.encoding == "rgb8":
        write_png(args.save, bytes(image.data), image.width, image.height)
        print(f"saved camera snapshot to {args.save}")

    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()

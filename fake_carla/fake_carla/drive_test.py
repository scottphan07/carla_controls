"""Drive the car with controls_node, standing in for planning and perception.

    ros2 run fake_carla drive_test                          # 30 m straight ahead at 5 m/s
    ros2 run fake_carla drive_test --distance 60 --speed 8
    ros2 run fake_carla drive_test --no-lidar               # report "nothing ahead" instead of using lidar

Needs the sim (real or fake) and controls_node running, e.g.
    ros2 launch carla_controls sim.launch.py                                   (real CARLA)
    ros2 launch fake_carla fake_sim.launch.py controls:=true wait_for_control:=true cruise_speed:=0.0

What it publishes, stamped with sim time (controls treats anything else as stale):
  /planning/target               controls_msgs/ControlTarget   goal --distance m ahead of the start;
                                                               speed 0 + brake_request within 3 m of it
  /perception/obstacle_distance  sensor_msgs/Range             gap from our front bumper to the nearest
                                                               lidar point in our lane (from
                                                               /sensors/lidar), +inf if clear

It prints progress once a second and exits 0 when the car has stopped at the goal for 2 s,
1 on timeout. If something blocks the lane, controls should stop for it (stop_reason obstacle)
and the run times out; that's the obstacle behavior working, not a crash.
"""
import argparse
import math
import sys

import rclpy
from controls_msgs.msg import ControlStatus, ControlTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, Range
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

from .drive_logic import DriveTest, obstacle_distance


class DriveTestNode(Node):
    def __init__(self, test: DriveTest, use_lidar: bool):
        super().__init__("drive_test", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        self.test = test
        self.use_lidar = use_lidar
        self.status = None
        self.obstacle = None
        self.last_print = -1.0
        self.done = None

        self.target_pub = self.create_publisher(ControlTarget, "/planning/target", 10)
        self.range_pub = self.create_publisher(Range, "/perception/obstacle_distance", 10)
        self.create_subscription(Odometry, "/carla/ego_vehicle/odometry", self.on_odometry, 10)
        self.create_subscription(ControlStatus, "/controls/status", self.on_status, 10)
        if use_lidar:
            self.create_subscription(PointCloud2, "/sensors/lidar", self.on_lidar, 10)

    def on_status(self, msg):
        self.status = msg

    def on_lidar(self, msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"))
        self.obstacle = obstacle_distance(pts)
        self.range_pub.publish(self._range(msg.header, self.obstacle))

    def on_odometry(self, msg: Odometry):
        if self.done is not None:
            return
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        p, q, v = msg.pose.pose.position, msg.pose.pose.orientation, msg.twist.twist.linear
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        speed = math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2)

        desired, brake = self.test.update(t, p.x, p.y, yaw, speed)
        header = Header(stamp=msg.header.stamp, frame_id="map")
        self.target_pub.publish(ControlTarget(
            header=header, desired_speed=float(desired), target_x=float(self.test.goal[0]),
            target_y=float(self.test.goal[1]), has_target_yaw=False, target_yaw=0.0,
            brake_request=brake))
        if not self.use_lidar:
            self.range_pub.publish(self._range(header, None))

        if t - self.last_print >= 1.0:
            self.last_print = t
            s = self.status
            ctl = (f"throttle {s.throttle:.2f} brake {s.brake:.2f} steer {s.steer:+.2f} "
                   f"stop={s.stop_reason or '-'}") if s else "no /controls/status yet"
            ahead = f"{self.obstacle:.1f} m" if self.obstacle is not None else "clear"
            print(f"t={t - self.test.start_time:5.1f}s  to goal {self.test.distance_to_goal(p.x, p.y):5.1f} m  "
                  f"speed {speed:4.1f} m/s  ahead {ahead:>7}  | {ctl}", flush=True)

        self.done = self.test.result(t)
        if self.done is not None:
            if self.done:
                print(f"PASS: stopped {self.test.distance_to_goal(p.x, p.y):.1f} m from the goal after "
                      f"{t - self.test.start_time:.1f} s, top speed {self.test.max_speed:.1f} m/s")
            elif self.status and self.status.stop_reason == "obstacle":
                print(f"STOPPED FOR OBSTACLE: something {self.obstacle:.1f} m ahead of the bumper, "
                      f"{self.test.distance_to_goal(p.x, p.y):.1f} m short of the goal. Controls' obstacle "
                      f"stop works; clear the lane or use a shorter --distance to test reaching the goal.")
            else:
                print(f"FAIL: didn't stop at the goal within {self.test.timeout:.0f} s "
                      f"(last stop_reason: {self.status.stop_reason if self.status else 'none'})")

    @staticmethod
    def _range(header, distance):
        return Range(header=header, radiation_type=Range.INFRARED, field_of_view=0.0,
                     min_range=0.0, max_range=50.0,
                     range=float(distance) if distance is not None else math.inf)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--distance", type=float, default=30.0, help="goal this many meters straight ahead")
    parser.add_argument("--speed", type=float, default=5.0, help="requested speed, m/s")
    parser.add_argument("--timeout", type=float, default=60.0, help="give up after this many sim seconds")
    parser.add_argument("--no-lidar", action="store_true", help="always report the lane as clear")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = DriveTestNode(DriveTest(distance=args.distance, speed=args.speed, timeout=args.timeout),
                         use_lidar=not args.no_lidar)
    print(f"goal: {args.distance:.0f} m straight ahead at {args.speed:.1f} m/s; waiting for odometry ...")
    try:
        while rclpy.ok() and node.done is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    ok = bool(node.done)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()

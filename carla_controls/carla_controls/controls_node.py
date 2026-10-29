"""ROS 2 node: the endpoints of the controls project.

Inputs (subscriptions)
  /carla/ego_vehicle/odometry             nav_msgs/Odometry                  CARLA ROS bridge, once per sim tick
  /planning/target                        controls_msgs/ControlTarget        planning
  /perception/obstacle_distance           sensor_msgs/Range                  perception (+inf = nothing ahead)
  /controls/emergency_stop                std_msgs/Bool                      anyone

Outputs (publishers)
  /carla/ego_vehicle/vehicle_control_cmd  carla_msgs/CarlaEgoVehicleControl  CARLA ROS bridge
  /controls/status                        controls_msgs/ControlStatus        planning and perception

The control loop runs in the odometry callback. The bridge publishes odometry
once per sim tick, so controls runs exactly once per tick. Start the bridge with
synchronous_mode_wait_for_vehicle_control_command:=True so it waits for our
command before ticking again.

TODO:
- Read the real max steering angle from /carla/ego_vehicle/vehicle_info
- Brake when shutting down (CARLA keeps applying the last command it got)
- Detect stale odometry (bridge stopped)
"""
import math

import rclpy
from carla_msgs.msg import CarlaEgoVehicleControl
from controls_msgs.msg import ControlStatus, ControlTarget
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Range
from std_msgs.msg import Bool

from .controller import Controller
from .messages import PerceptionInput, PlanningInput, VehicleState

DT = 0.05  # [s] must match the bridge's fixed_delta_seconds


class ControlsNode(Node):
    def __init__(self):
        super().__init__("controls")
        self.controller = Controller(dt=DT)
        self.plan = None  # latest PlanningInput
        self.perception = None  # latest PerceptionInput

        self.create_subscription(Odometry, "/carla/ego_vehicle/odometry", self.on_odometry, 10)
        self.create_subscription(ControlTarget, "/planning/target", self.on_target, 10)
        self.create_subscription(Range, "/perception/obstacle_distance", self.on_obstacle, 10)
        self.create_subscription(Bool, "/controls/emergency_stop", self.on_emergency_stop, 10)

        self.command_pub = self.create_publisher(
            CarlaEgoVehicleControl, "/carla/ego_vehicle/vehicle_control_cmd", 10)
        self.status_pub = self.create_publisher(ControlStatus, "/controls/status", 10)

    # ------------------------------------------------ inputs: just store the latest message
    def on_target(self, msg: ControlTarget):
        self.plan = PlanningInput(
            timestamp=Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
            desired_speed=msg.desired_speed,
            target_x=msg.target_x,
            target_y=msg.target_y,
            target_yaw=msg.target_yaw if msg.has_target_yaw else None,
            brake_request=msg.brake_request,
        )

    def on_obstacle(self, msg: Range):
        self.perception = PerceptionInput(
            timestamp=Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
            obstacle_distance=msg.range if math.isfinite(msg.range) else None,
        )

    def on_emergency_stop(self, msg: Bool):
        self.controller.emergency_stop = msg.data

    # ------------------------------------------------ the control loop: once per sim tick
    def on_odometry(self, msg: Odometry):
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        state = VehicleState(
            timestamp=Time.from_msg(msg.header.stamp).nanoseconds * 1e-9,
            x=msg.pose.pose.position.x,
            y=msg.pose.pose.position.y,
            yaw=math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)),
            speed=math.sqrt(v.x ** 2 + v.y ** 2 + v.z ** 2),
        )

        output = self.controller.step(state, self.plan, self.perception)
        cmd = output.command

        self.command_pub.publish(CarlaEgoVehicleControl(
            header=msg.header,
            throttle=float(cmd.throttle),
            steer=float(-cmd.steer),  # ours: + = left (ROS); bridge/CARLA: + = right
            brake=float(cmd.brake),
            hand_brake=cmd.hand_brake,
            reverse=cmd.reverse,
        ))
        self.status_pub.publish(ControlStatus(
            header=msg.header,
            throttle=float(cmd.throttle),
            steer=float(cmd.steer),
            brake=float(cmd.brake),
            stop_reason=output.stop_reason.value if output.stop_reason else "",
        ))


def main():
    rclpy.init()
    node = ControlsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

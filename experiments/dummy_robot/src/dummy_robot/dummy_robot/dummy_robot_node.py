#!/usr/bin/env python3
"""Lanelet-aware fake robot.

Publishes the same /dummy_robot/odom, /dummy_robot/battery_percent,
/dummy_robot/operational_state topics the old deterministic (C++) version
did -- robot_state_aggregator and zenoh_bridge_core need no changes to
work with this. Subscribes to /dummy_robot/goal (geometry_msgs/PoseStamped,
published by NavigationGatewayComponent on a navigate_to_goal Zenoh
command) and /dummy_robot/cancel (std_msgs/Header, on cancel_navigation).
On a new goal it plans a route across the lanelet2 map to it (see
lanelet_router.py) and drives toward it at a constant speed, publishing
operational_state idle -> navigating -> arrived; on cancel it halts exactly
where it is. Not a physics simulator -- swapping this for Autoware or a
real robot later means pointing robot_state_aggregator at different
topics, not changing anything downstream.
"""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, Header, String

from dummy_robot.lanelet_router import LaneletRouter

ARRIVAL_TOLERANCE_M = 0.3


class DummyRobotNode(Node):
    def __init__(self) -> None:
        super().__init__("dummy_robot_node")

        self.declare_parameter("update_rate_hz", 10.0)
        self.declare_parameter("x", -55.01039897722149)
        self.declare_parameter("y", 65.38078806758735)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("battery_cycle_sec", 600.0)
        self.declare_parameter("speed_mps", 1.5)
        self.declare_parameter("map_osm_path", "/opt/zenoh_bridge_files/maps/lanelet2_map.osm")
        self.declare_parameter("map_projector_path", "/opt/zenoh_bridge_files/maps/map_projector_info.yaml")

        self._x = self.get_parameter("x").value
        self._y = self.get_parameter("y").value
        self._yaw = self.get_parameter("yaw").value
        self._battery_cycle_sec = self.get_parameter("battery_cycle_sec").value
        self._speed_mps = self.get_parameter("speed_mps").value
        update_rate_hz = self.get_parameter("update_rate_hz").value

        self._operational_state = "idle"
        self._path: list[tuple[float, float]] = []
        self._path_index = 0
        # Sequence guard: /dummy_robot/goal and /dummy_robot/cancel are two
        # separate topics/subscriptions, and DDS only guarantees in-order
        # delivery *within* a topic, not across two of them -- without this,
        # a fast pause-then-resume could have the resume's goal processed
        # before the pause's cancel, silently re-freezing a robot that just
        # got a legitimate new goal. NavigationGatewayComponent stamps both
        # messages' header.stamp.sec with the same monotonic counter (a
        # sequence number, not wall-clock time); ignore anything not newer
        # than the last one actually applied. Starts at 0 to accept the
        # gateway's first command (seq 1).
        self._last_seq = 0

        try:
            self._router = LaneletRouter(
                self.get_parameter("map_osm_path").value,
                self.get_parameter("map_projector_path").value,
            )
        except Exception as exc:
            self._router = None
            self.get_logger().error(f"LaneletRouter unavailable; goals will be rejected: {exc}")

        self._odom_pub = self.create_publisher(Odometry, "/dummy_robot/odom", 10)
        self._battery_pub = self.create_publisher(Float32, "/dummy_robot/battery_percent", 10)
        self._state_pub = self.create_publisher(String, "/dummy_robot/operational_state", 10)
        self.create_subscription(PoseStamped, "/dummy_robot/goal", self._on_goal, 10)
        self.create_subscription(Header, "/dummy_robot/cancel", self._on_cancel, 10)

        self._start_time = time.monotonic()
        self._last_tick = self._start_time
        self.create_timer(1.0 / update_rate_hz, self._tick)

        self.get_logger().info(f"dummy_robot_node started at ({self._x:.2f}, {self._y:.2f})")

    def _on_goal(self, msg: PoseStamped) -> None:
        seq = msg.header.stamp.sec
        if seq <= self._last_seq:
            self.get_logger().warning(f"Ignoring out-of-order/stale goal (seq={seq} <= {self._last_seq})")
            return
        self._last_seq = seq

        gx, gy = msg.pose.position.x, msg.pose.position.y
        if self._router is None:
            self.get_logger().error("Rejecting goal: map/lanelet2 unavailable")
            return

        path = self._router.plan_route(self._x, self._y, gx, gy)
        if not path:
            self.get_logger().warning(f"No route to goal ({gx:.2f}, {gy:.2f}); ignoring")
            return

        self._path = path
        self._path_index = 0
        self._operational_state = "navigating"
        self.get_logger().info(f"New goal ({gx:.2f}, {gy:.2f}): {len(path)}-point route")

    def _on_cancel(self, msg: Header) -> None:
        seq = msg.stamp.sec
        if seq <= self._last_seq:
            self.get_logger().warning(f"Ignoring out-of-order/stale cancel (seq={seq} <= {self._last_seq})")
            return
        self._last_seq = seq

        self._path = []
        self._path_index = 0
        self._operational_state = "idle"
        self.get_logger().info(f"Navigation cancelled; halted at ({self._x:.2f}, {self._y:.2f})")

    def _tick(self) -> None:
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        self._advance(dt)
        self._publish_odom()
        self._publish_battery(now)
        self._publish_operational_state()

    def _advance(self, dt: float) -> None:
        if self._operational_state != "navigating" or not self._path:
            return

        remaining = self._speed_mps * dt
        while remaining > 0.0 and self._path_index < len(self._path):
            tx, ty = self._path[self._path_index]
            dist = math.hypot(tx - self._x, ty - self._y)
            if dist <= ARRIVAL_TOLERANCE_M:
                self._path_index += 1
                continue
            step = min(remaining, dist)
            self._yaw = math.atan2(ty - self._y, tx - self._x)
            self._x += (tx - self._x) / dist * step
            self._y += (ty - self._y) / dist * step
            remaining -= step
            if step < dist:
                break

        if self._path_index >= len(self._path):
            self._operational_state = "arrived"
            self._path = []

    def _publish_odom(self) -> None:
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        speed = self._speed_mps if self._operational_state == "navigating" else 0.0
        msg.twist.twist.linear.x = speed * math.cos(self._yaw)
        msg.twist.twist.linear.y = speed * math.sin(self._yaw)
        self._odom_pub.publish(msg)

    def _publish_battery(self, now: float) -> None:
        elapsed = now - self._start_time
        cycle_pos = math.fmod(elapsed, self._battery_cycle_sec) / self._battery_cycle_sec
        msg = Float32()
        msg.data = 100.0 - cycle_pos * 80.0  # cycles 100 -> 20, deterministic
        self._battery_pub.publish(msg)

    def _publish_operational_state(self) -> None:
        msg = String()
        msg.data = self._operational_state
        self._state_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = DummyRobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

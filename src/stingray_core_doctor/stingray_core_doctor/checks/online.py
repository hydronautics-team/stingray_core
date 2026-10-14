"""Read-only observations of an already-running sensor stack.

This is intentionally not a supervisor: it never starts, stops, restarts, or
reconfigures a node and it never grants permission to arm the vehicle.
"""

from dataclasses import dataclass
from typing import Any

import rclpy
from dvl_msgs.msg import DVL
from geometry_msgs.msg import PointStamped
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from stingray_core_doctor.checks.base import BaseCheck
from stingray_core_doctor.types import CheckResult, CheckStatus


@dataclass(frozen=True)
class Observation:
    """Minimal evidence collected from one topic."""

    received: bool
    message: Any = None


class OnlineSensorCheck(BaseCheck):
    """Observe existing sensor publishers without owning them."""

    category_name = "Online sensor observations"

    def run(self) -> list[CheckResult]:
        """Inspect streams without changing the running ROS graph."""
        rclpy.init()
        try:
            return [
                self._check_imu(),
                self._check_pressure(),
                self._check_dvl(),
            ]
        finally:
            rclpy.shutdown()

    def _observe(self, topic_name: str, msg_type, timeout: float) -> Observation:
        message = None

        def callback(received_message):
            nonlocal message
            message = received_message

        node = Node("stingray_core_doctor_observer")
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        started_at = clock.now()
        subscription = node.create_subscription(
            msg_type, topic_name, callback, qos_profile_sensor_data
        )
        try:
            while message is None:
                elapsed = (clock.now() - started_at).nanoseconds / 1e9
                if elapsed >= timeout:
                    break
                rclpy.spin_once(node, timeout_sec=min(0.1, timeout - elapsed))
            return Observation(message is not None, message)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()

    def _check_imu(self) -> CheckResult:
        topic = self.config.imu
        observation = self._observe(topic.name, Imu, topic.timeout_sec)
        if not observation.received:
            return CheckResult(
                "IMU stream",
                CheckStatus.FAIL,
                f"No messages received on {topic.name}.",
                "Check the VectorNav driver and adapter diagnostics.",
            )
        return CheckResult("IMU stream", CheckStatus.OK)

    def _check_pressure(self) -> CheckResult:
        topic = self.config.pressure
        observation = self._observe(topic.name, PointStamped, topic.timeout_sec)
        if not observation.received:
            return CheckResult(
                "Depth stream",
                CheckStatus.FAIL,
                f"No messages received on {topic.name}.",
                "Check both pressure_link and pressure_sensor diagnostics.",
            )
        return CheckResult("Depth stream", CheckStatus.OK)

    def _check_dvl(self) -> CheckResult:
        topic = self.config.dvl
        observation = self._observe(topic.name, DVL, topic.timeout_sec)
        if not observation.received:
            return CheckResult(
                "DVL stream",
                CheckStatus.FAIL,
                f"No messages received on {topic.name}.",
                "Check the DVL network and driver diagnostics.",
            )
        if not observation.message.velocity_valid:
            return CheckResult(
                "DVL stream",
                CheckStatus.WARN,
                "The DVL is publishing, but velocity is not valid.",
                "No bottom lock can be normal while the vehicle is out of water.",
            )
        return CheckResult("DVL stream", CheckStatus.OK)

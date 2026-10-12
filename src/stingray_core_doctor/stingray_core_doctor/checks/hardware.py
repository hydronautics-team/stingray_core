import os
import signal
import subprocess

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data

from vectornav_msgs.msg import ImuGroup
from geometry_msgs.msg import PointStamped
from dvl_msgs.msg import DVL

from stingray_core_doctor.checks.base import BaseCheck
from stingray_core_doctor.types import CheckResult, CheckStatus

class HardwareCheck(BaseCheck):

    def __init__(self, config):
        super().__init__(config)
        self.category_name = "Hardware"
        self.processes: list[subprocess.Popen] = []

    def run(self) -> list[CheckResult]:
        results = []

        rclpy.init()

        try:
            results.extend(self.check_vectornav())
            results.extend(self.check_pressure())
            results.extend(self.check_dvl())
            # results.extend(self.check_camera())

            return results
        finally:
            rclpy.shutdown()


    def wait_for_topic(
        self,
        topic_name: str,
        msg_type,
        timeout: float = 3.0,
    ) -> bool:
        received = False

        def callback(msg):
            nonlocal received
            if msg is not None:
                received = True

        node = Node("stingray_core_doctor_topic_check")
        clock = Clock(clock_type=ClockType.STEADY_TIME)
        start_time = clock.now()

        try:
            node.create_subscription(
                msg_type,
                topic_name,
                callback,
                qos_profile_sensor_data,
            )

            while not received:
                elapsed = (clock.now() - start_time).nanoseconds / 1e9

                if elapsed >= timeout:
                    break

                rclpy.spin_once(
                    node,
                    timeout_sec=min(0.1, timeout - elapsed),
                )

            return received

        finally:
            node.destroy_node()
        
    def start_sensor(
        self,
        package: str,
        launch_file: str,
        launch_arguments: Optional[dict[str, str]] = None,
    ) -> bool:
        command = ["ros2", "launch", package, launch_file]

        if launch_arguments:
            for name, value in launch_arguments.items():
                command.append(f"{name}:={value}")

        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                start_new_session=True,
            )
        except (OSError, subprocess.SubprocessError):
            return False

        self.processes.append(process)

        return process.poll() is None

    def stop_sensor(self, process: subprocess.Popen) -> None:
        if process.poll() is not None:
            return

        try:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(process.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

            process.wait()
        except ProcessLookupError:
            pass
    
    def check_vectornav(self) -> list[CheckResult]:
        if not self.start_sensor("vectornav", "vectornav.launch.py"):
            return [
                CheckResult(
                    name="VectorNav",
                    status=CheckStatus.FAIL,
                    message="Failed to start VectorNav node.",
                    hint="Check UART connection, /dev/ttyAMA4 permissions, launch file.",
                )
            ]
        try:
            received = self.wait_for_topic(
                self.config.vectornav_topic,
                ImuGroup,
                timeout=self.config.timeout,
            )

            if received:
                return [
                    CheckResult(
                        name="VectorNav",
                        status=CheckStatus.OK,
                    )
                ]

            return [
                CheckResult(
                    name="VectorNav",
                    status=CheckStatus.FAIL,
                    message=f"No IMU messages received within {self.config.timeout} seconds.",
                    hint="Check VectorNav power, UART connection and driver configuration.",
                )
            ]
        finally:
            self.stop_sensor(self.processes[-1])

    def check_pressure(self) -> list[CheckResult]:
        if not self.start_sensor(
            "pressure_sensor",
            self.config.pressure_launch,
        ):
            return [
                CheckResult(
                    name="Pressure sensor",
                    status=CheckStatus.FAIL,
                    message="Failed to start pressure sensor launch file.",
                    hint=(
                        "Check the pressure_sensor package and its launch file."
                    ),
                )
            ]
        
        try:
            received = self.wait_for_topic(
                self.config.pressure_topic,
                PointStamped,
                timeout=self.config.timeout,
            )

            if received:
                return [
                    CheckResult(
                        name="Pressure sensor",
                        status=CheckStatus.OK,
                    )
                ]

            return [
                CheckResult(
                    name="Pressure sensor",
                    status=CheckStatus.FAIL,
                    message=(
                        "No messages received on "
                        "/stingray_core/pressure_sensor/depth "
                        f"within {self.config.timeout} seconds."
                    ),
                    hint=(
                        "Check pressure sensor power, hardware connection, "
                        "and the /data_raw input topic."
                    ),
                )
            ]
        finally:
            self.stop_sensor(self.processes[-1])

    def check_dvl(self) -> list[CheckResult]:
        if not self.start_sensor(
            "dvl_a50",
            self.config.dvl_launch,
            {"ip_address": self.config.dvl_ip},
        ):
            return [
                CheckResult(
                    name="DVL",
                    status=CheckStatus.FAIL,
                    message="Failed to start DVL driver.",
                    hint="Check the launch file and ROS 2 package installation.",
                )
            ]

        try:
            received = self.wait_for_topic(
                self.config.dvl_topic,
                DVL,
                timeout=self.config.timeout,
            )

            if received:
                return [
                    CheckResult(
                        name="DVL",
                        status=CheckStatus.OK,
                    )
                ]

            return [
                CheckResult(
                    name="DVL",
                    status=CheckStatus.FAIL,
                    message=f"No DVL messages received within {self.config.timeout} seconds.",
                    hint="Check DVL power, network connection and IP address.",
                )
            ]

        finally:
            self.stop_sensor(self.processes[-1])

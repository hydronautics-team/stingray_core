#!/usr/bin/env python3

from __future__ import annotations

import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ManipulatorSubNode(Node):
    """ROS2 node for manipulator PWM control via serial port.

    Subscription:
      - `cmd` (std_msgs/Int32)
        PWM value from 0 to 99.

    Parameters:
      - port (str): serial device path, e.g. '/dev/ttyUSB0'
      - baud_rate (int): serial baud rate, e.g. 115200
      - min_pwm (int): minimum allowed PWM value
      - max_pwm (int): maximum allowed PWM value
    """

    def __init__(self) -> None:
        super().__init__("manipulator_device_node")

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("min_pwm", 0)
        self.declare_parameter("max_pwm", 99)

        self.port: str = (
            self.get_parameter("port")
            .get_parameter_value()
            .string_value
        )

        self.baud_rate: int = (
            self.get_parameter("baud_rate")
            .get_parameter_value()
            .integer_value
        )

        self.min_pwm: int = (
            self.get_parameter("min_pwm")
            .get_parameter_value()
            .integer_value
        )

        self.max_pwm: int = (
            self.get_parameter("max_pwm")
            .get_parameter_value()
            .integer_value
        )

        self._check_serial_port()

        self.subscription = self.create_subscription(
            Int32,
            "cmd",
            self._pwm_callback,
            10,
        )

        self.get_logger().info(
            f"Manipulator node started. "
            f"Serial: {self.port} @ {self.baud_rate} baud"
        )

    def _check_serial_port(self) -> None:
        """Check that the configured serial port is accessible."""

        try:
            with serial.Serial(
                self.port,
                self.baud_rate,
                timeout=1,
            ):
                pass

            self.get_logger().info(
                f"Serial port ready: {self.port} "
                f"@ {self.baud_rate} baud"
            )

        except Exception as e:
            self.get_logger().error(
                f"Cannot access serial port {self.port}: {e}"
            )

    def _pwm_callback(self, msg: Int32) -> None:
        """Receive PWM command and send it to manipulator over UART."""

        pwm_value = max(
            self.min_pwm,
            min(self.max_pwm, msg.data),
        )

        self.get_logger().info(
            f"Received PWM command: {msg.data} -> {pwm_value}"
        )

        tens = pwm_value // 10
        ones = pwm_value % 10

        bytes_to_send = f"{tens}{ones}".encode("ascii")

        try:
            with serial.Serial(
                self.port,
                self.baud_rate,
                timeout=1,
            ) as serial_port:
                serial_port.write(bytes_to_send)

            self.get_logger().info(
                f"Sent to UART: {list(bytes_to_send)}"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to write to serial port: {e}"
            )

def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ManipulatorSubNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

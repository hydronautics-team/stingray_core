#!/usr/bin/env python3

from __future__ import annotations

import time

import gpiod
import serial

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class ManipulatorSubNode(Node):
    """ROS2 node for manipulator control over RS-485.

    Subscription:
      - `cmd` (std_msgs/Int32)
        PWM command in range 0..99.

    Parameters:
      - port_path (str): UART device, e.g. /dev/ttyAMA2
      - baud_rate (int): UART baud rate
      - rs485_gpio_chip (str): GPIO chip, e.g. gpiochip0
      - rs485_gpio (int): GPIO controlling RS-485 DE/RE
      - min_pwm (int): minimum PWM command
      - max_pwm (int): maximum PWM command
      - tx_delay (float): delay after enabling RS-485 transmitter
    """

    def __init__(self) -> None:
        super().__init__("manipulator_sub_node")

        self.declare_parameter("port_path", "/dev/ttyAMA2")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("rs485_gpio_chip", "gpiochip0")
        self.declare_parameter("rs485_gpio", 6)
        self.declare_parameter("min_pwm", 0)
        self.declare_parameter("max_pwm", 99)
        self.declare_parameter("tx_delay", 0.001)

        self.port_path = self.get_parameter("port_path").value
        self.baud_rate = self.get_parameter("baud_rate").value
        self.rs485_gpio_chip = self.get_parameter("rs485_gpio_chip").value
        self.rs485_gpio = self.get_parameter("rs485_gpio").value
        self.min_pwm = self.get_parameter("min_pwm").value
        self.max_pwm = self.get_parameter("max_pwm").value
        self.tx_delay = self.get_parameter("tx_delay").value

        self._chip = None
        self._line = None
        self._serial = None

        self._init_gpio()
        self._init_serial()

        self.subscription = self.create_subscription(
            Int32,
            "cmd",
            self._pwm_callback,
            10,
        )

        self.get_logger().info(
            "Manipulator RS-485 node started"
        )

        self.get_logger().info(
            f"UART: {self.port_path} @ {self.baud_rate}"
        )

        self.get_logger().info(
            f"RS-485 DE/RE: "
            f"{self.rs485_gpio_chip}:{self.rs485_gpio}"
        )

        self.get_logger().info(
            "Command topic: ~/cmd"
        )

    def _init_gpio(self) -> None:
        """Initialize RS-485 direction GPIO."""

        try:
            self._chip = gpiod.Chip(self.rs485_gpio_chip)

            self._line = self._chip.get_line(
                self.rs485_gpio
            )

            self._line.request(
                consumer="manipulator_rs485",
                type=gpiod.LINE_REQ_DIR_OUT,
                default_vals=[0],
            )

            # LOW = receiver mode / transmitter disabled
            self._line.set_value(0)

            self.get_logger().info(
                f"GPIO initialized: "
                f"{self.rs485_gpio_chip}:{self.rs485_gpio} = LOW"
            )

        except Exception as e:
            self.get_logger().fatal(
                f"Failed to initialize RS-485 GPIO: {e}"
            )
            raise

    def _init_serial(self) -> None:
        """Open UART used by RS-485 transceiver."""

        try:
            self._serial = serial.Serial(
                port=self.port_path,
                baudrate=self.baud_rate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                write_timeout=1,
            )

            self.get_logger().info(
                f"UART opened: "
                f"{self.port_path} @ {self.baud_rate} 8N1"
            )

        except Exception as e:
            self.get_logger().fatal(
                f"Failed to open UART {self.port_path}: {e}"
            )
            raise

    def _pwm_callback(self, msg: Int32) -> None:
        """Send PWM command to manipulator."""

        pwm_value = max(
            self.min_pwm,
            min(self.max_pwm, msg.data),
        )

        if msg.data != pwm_value:
            self.get_logger().warning(
                f"PWM {msg.data} clamped to {pwm_value}"
            )

        # Convert 0..99 to exactly two ASCII characters.
        data = f"{pwm_value:02d}".encode("ascii")

        self.get_logger().info(
            f"Sending PWM={pwm_value} "
            f"data={data!r}"
        )

        try:
            # Enable RS-485 transmitter.
            self._line.set_value(1)

            time.sleep(self.tx_delay)

            # Send data.
            self._serial.write(data)
            self._serial.flush()

            # Wait until pyserial buffer is empty.
            while self._serial.out_waiting > 0:
                time.sleep(0.001)

            # Give UART time to finish the final bits.
            # At 115200 baud, 2 bytes are very short,
            # but we keep a small guard delay.
            time.sleep(0.001)

        except Exception as e:
            self.get_logger().error(
                f"RS-485 transmission failed: {e}"
            )

        finally:
            # Disable RS-485 transmitter.
            self._line.set_value(0)

    def destroy_node(self) -> bool:
        """Release UART and GPIO resources."""

        try:
            if self._line is not None:
                self._line.set_value(0)
                self._line.release()

        except Exception as e:
            self.get_logger().warning(
                f"Failed to release GPIO: {e}"
            )

        try:
            if self._serial is not None:
                self._serial.close()

        except Exception as e:
            self.get_logger().warning(
                f"Failed to close UART: {e}"
            )

        try:
            if self._chip is not None:
                self._chip.close()

        except Exception as e:
            self.get_logger().warning(
                f"Failed to close GPIO chip: {e}"
            )

        return super().destroy_node()


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
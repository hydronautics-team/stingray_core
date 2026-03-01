#!/usr/bin/env python3

from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class PowerGpioNode(Node):
    """ROS2 node to drive one GPIO line via python-libgpiod.

    Service:
      - `~/set_power` (std_srvs/SetBool)
        * data=True  -> logical ON
        * data=False -> logical OFF

    Parameters:
      - gpio_chip (str): e.g. 'gpiochip0'
      - gpio_line (int): BCM line number (e.g. 17). If -1 -> node disabled.
      - active_high (bool): if False, logic is inverted
      - default_on (bool): applied right after requesting the line
      - consumer (str): gpiod consumer string
    """

    def __init__(self) -> None:
        super().__init__("power_gpio_node")

        self.declare_parameter("gpio_chip", "gpiochip0")
        self.declare_parameter("gpio_line", -1)
        self.declare_parameter("active_high", True)
        self.declare_parameter("default_on", True)
        self.declare_parameter("consumer", "power_control/power_gpio_node")

        self.gpio_chip: str = (
            self.get_parameter("gpio_chip").get_parameter_value().string_value
        )
        self.gpio_line: int = (
            self.get_parameter("gpio_line").get_parameter_value().integer_value
        )
        self.active_high: bool = (
            self.get_parameter("active_high").get_parameter_value().bool_value
        )
        self.default_on: bool = (
            self.get_parameter("default_on").get_parameter_value().bool_value
        )
        self.consumer: str = (
            self.get_parameter("consumer").get_parameter_value().string_value
        )

        self._ready = False
        self._gpiod = None
        self._chip = None
        self._line = None

        self.srv = self.create_service(SetBool, "~/set_power", self._handle_set_power)
        self._init_gpio()

    def _init_gpio(self) -> None:
        try:
            import gpiod  # type: ignore

            self._gpiod = gpiod
        except Exception as e:
            self.get_logger().error(
                f"Python module 'gpiod' is not available: {e}. "
                "Install python3-gpiod (or equivalent) and restart node."
            )
            self._ready = False
            return

        if self.gpio_line < 0:
            self.get_logger().error(
                "Parameter 'gpio_line' is not set (<0). "
                "Set it to BCM line number (e.g. 17) and restart node."
            )
            self._ready = False
            return

        try:
            self._chip = self._gpiod.Chip(self.gpio_chip)
            self._line = self._chip.get_line(self.gpio_line)
            self._line.request(
                consumer=self.consumer, type=self._gpiod.LINE_REQ_DIR_OUT
            )
            self._set_power(self.default_on)
            self._ready = True

            self.get_logger().info(
                f"GPIO ready: chip={self.gpio_chip}, line={self.gpio_line}, "
                f"active_high={self.active_high}, default_on={self.default_on}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Failed to init GPIO (chip={self.gpio_chip}, line={self.gpio_line}): {e}"
            )
            self._ready = False

    def _logical_to_level(self, on: bool) -> int:
        if self.active_high:
            return 1 if on else 0
        return 0 if on else 1

    def _set_power(self, on: bool) -> None:
        if self._line is None:
            raise RuntimeError("GPIO line is not requested")
        self._line.set_value(self._logical_to_level(on))

    def _handle_set_power(
        self, request: SetBool.Request, response: SetBool.Response
    ) -> SetBool.Response:
        if not self._ready:
            response.success = False
            response.message = (
                "GPIO is not ready (gpiod missing or gpio_line not configured)."
            )
            return response

        try:
            self._set_power(bool(request.data))
            response.success = True
            response.message = "ON" if request.data else "OFF"
            return response
        except Exception as e:
            response.success = False
            response.message = f"Failed to set GPIO: {e}"
            return response


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = PowerGpioNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

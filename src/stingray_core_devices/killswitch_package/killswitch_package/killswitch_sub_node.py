#!/usr/bin/env python3
import gpiod
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KillswitchSubscriber(Node):
    def __init__(self):
        super().__init__("killswitch_subscriber")
        self.last_state = None
        self.chip = gpiod.Chip('gpiochip0')
        self.line = self.chip.get_line(20)
        self.line.request(consumer='killswitch', type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])

        self.subscription = self.create_subscription(
            Bool, "/stingray_core/safety/kill_switch", self.pwm_callback, 10
        )
        self.get_logger().info("Нода-подписчик запущена!")

    def pwm_callback(self, msg):
        if msg.data != self.last_state:
            self.last_state = msg.data
            if msg.data:
                self.line.set_value(1)
                self.get_logger().info("Килсвитч активирован!")
            else:
                self.line.set_value(0)
                self.get_logger().info("Килсвитч деактивирован!")
            


def main(args=None):
    rclpy.init(args=args)
    node = KillswitchSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Нода остановлена пользователем.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

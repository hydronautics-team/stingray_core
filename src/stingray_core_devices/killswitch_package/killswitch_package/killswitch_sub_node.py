#!/usr/bin/env python3
import subprocess
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class KillswitchSubscriber(Node):
    def __init__(self):
        super().__init__("killswitch_subscriber")
        self.last_state = None

        self.subscription = self.create_subscription(
            Bool, "/stingray_core/safety/kill_switch", self.pwm_callback, 10
        )
        self.get_logger().info("Нода-подписчик запущена!")

    def pwm_callback(self, msg):
        if msg.data != self.last_state:
            self.last_state = msg.data
            if msg.data:
                subprocess.run(['pinctrl', 'set', '20', 'op', 'dh'], check=False)
                self.get_logger().info("Килсвитч активирован!")
            else:
                subprocess.run(['pinctrl', 'set', '20', 'op', 'dl'], check=False)   
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

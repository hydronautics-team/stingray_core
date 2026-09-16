#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class ManipulatorSubscriber(Node):
    def __init__(self):
        super().__init__('manipulator_subscriber')
        self.port_path = '/dev/ttyUSB0'
        
        try:
            with open(self.port_path, 'wb') as f:
                pass
            self.get_logger().info(f"Порт {self.port_path} успешно найден.")
        except Exception as e:
            self.get_logger().error(f"Не удалось получить доступ к {self.port_path}: {e}")
            
        self.subscription = self.create_subscription(
            Int32,
            '/stingray_core/device/manipulator/cmd',
            self.pwm_callback,
            10
        )
        self.get_logger().info("Нода-подписчик запущена! Ожидаю поток данных ШИМ...")

    def pwm_callback(self, msg):
        pwm_value = msg.data
        self.get_logger().info(f"--> Принято значение ШИМ от пульта: [ {pwm_value} ]")
        
        val = max(0, min(99, pwm_value))
        tens = val // 10
        ones = val % 10
        
        bytes_to_send = f"{tens}{ones}".encode('ascii')
        try:
            with open(self.port_path, 'wb', buffering=0) as serial_port:
                serial_port.write(bytes_to_send)
                self.get_logger().info(f"Отправлено в ttyUSB0 (ASCII коды): {[b for b in bytes_to_send]}")
        except Exception as e:
            self.get_logger().error(f"Не удалось записать в порт: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ManipulatorSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Нода остановлена пользователем.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

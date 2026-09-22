#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial

class ManipulatorSubscriber(Node):
    def __init__(self):
        super().__init__('manipulator_subscriber')
        self.port_path = '/dev/ttyUSB0'
        self.baud_rate = 115200
        
        try:
            with serial.Serial(self.port_path, self.baud_rate, timeout=1) as ser:
                pass
            self.get_logger().info(f"[ОК] Порт {self.port_path} успешно найден на скорости {self.baud_rate}.")
        except Exception as e:
            self.get_logger().error(f"[ОШИБКА] Нет доступа к {self.port_path}: {e}")

        self.subscription = self.create_subscription(
            Int32,
            '/stingray_core/device/manipulator/cmd',
            self.pwm_callback,
            10
        )
        self.get_logger().info("Нода-подписчик запущена! Ожидаю ШИМ от Qt-пульта...")

    def pwm_callback(self, msg):
        pwm_value = msg.data
        self.get_logger().info(f"--> Принято значение ШИМ от пульта: [ {pwm_value} ]")

        val = max(0, min(99, pwm_value))
        tens = val // 10
        ones = val % 10
        bytes_to_send = f"{tens}{ones}".encode('ascii')
        
        try:
            with serial.Serial(self.port_path, self.baud_rate, timeout=1) as serial_port:
                serial_port.write(bytes_to_send)
            self.get_logger().info(f"    Отправлено в UART (115200): {[b for b in bytes_to_send]}")
        except Exception as e:
            self.get_logger().error(f"    [Ошибка записи]: {e}")

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

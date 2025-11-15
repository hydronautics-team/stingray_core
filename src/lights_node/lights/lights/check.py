import rclpy
from rclpy.node import Node
from lights_interfaces.msg import Brightness 

class LightDriver(Node):
    def __init__(self):
        super().__init__('light_driver')
        
        self.subscription = self.create_subscription(
            Brightness,              
            '/lights/brightness_cmd', 
            self.listener_callback,  
            10)                      
        
        self.get_logger().info('Драйвер света запущен и ожидает команд на /lights/brightness_cmd...')

    def listener_callback(self, msg):
        left = msg.left_value
        right = msg.right_value
        self.get_logger().info(f'-> КОМАНДА: Лев: {left:3} | Прав: {right:3}')

def main(args=None):
    rclpy.init(args=args)
    driver_node = LightDriver()
    rclpy.spin(driver_node)
    driver_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from lights_interfaces.msg import Brightness

from .code_light import BrightControl

class LanternControl(Node):
    def __init__(self):
        super().__init__('lantern_controller')
        
        self.lantern_pub = self.create_publisher(Brightness, '/lights/brightness_cmd', 10)
        
        self.logic_controller = BrightControl() 
        
        self.main_timer_period = 3.0
        self.state = 0
        self.main_timer = self.create_timer(self.main_timer_period, self.main_timer_callback)
        
        self.publish_time = 0.05 
        self.publish_timer = self.create_timer(self.publish_time, self.publish_value)
        
        self.stop_effect_timer = None
    
    def publish_value(self):
        left_value, right_value = self.logic_controller.bright()  
        
        msg = Brightness()
        msg.left_value = left_value
        msg.right_value = right_value
        
        self.lantern_pub.publish(msg)

    def back_to_solid(self):
        self.logic_controller.set_mode("SOLID")

    def set_mode(self, mode: str, period : float, duration : float):
        self.logic_controller.set_mode(mode, period=period)
        
        if self.stop_effect_timer:
            self.stop_effect_timer.cancel()
            self.stop_effect_timer = None
            
        self.stop_effect_timer = self.create_timer(duration, self.back_to_solid)
    

    def main_timer_callback(self):
        if self.state == 0:
            self.set_mode("ONE_BY_ONE", period=0.5, duration=5.0)
            
        elif self.state == 1:
            self.set_mode("BLINK", period=0.5, duration=5.0)
            
        elif self.state == 2:
            self.logic_controller.set_mode("SOLID")
            if self.stop_effect_timer:
                self.stop_effect_timer.cancel()
                self.stop_effect_timer = None

        elif self.state == 3:
            self.set_mode("FADE", period=0.5, duration=5.0)

        self.state = (self.state + 1) % 4
        

def main(args=None):
    rclpy.init(args=args)
    controller = LanternControl()
    
    rclpy.spin(controller)
    
    if controller.stop_effect_timer:
        controller.stop_effect_timer.cancel()
    if controller.publish_timer:
        controller.publish_timer.cancel()
    if controller.main_timer:
        controller.main_timer.cancel()
        
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
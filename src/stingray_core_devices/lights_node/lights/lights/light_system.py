import rclpy
from rclpy.node import Node
from rclpy.timer import Timer
from stingray_msgs.msg import Brightness, LightMode as LightModeMsg

from .code_light import BrightControl, LightMode

class LanternControl:
    def __init__(self):
        self.node = Node('lantern_controller')
        
        self.lantern_pub = self.node.create_publisher(
            Brightness, '/lights/brightness_cmd', 10
        )
        self.mode_sub = self.node.create_subscription(
            LightModeMsg,
            '/lights/set_mode',
            self.mode_callback,
            10
        )

        self.logic_controller = BrightControl() 
        
        self.main_timer_period = 3.0
        self.state = 0
        self.main_timer = self.node.create_timer(self.main_timer_period, self.main_timer_callback)
        
        self.publish_time = 0.05 
        self.publish_timer = self.node.create_timer(self.publish_time, self.publish_value)
        
        self.stop_effect_timer = None

    def mode_callback(self, msg):
        try:
            new_mode = LightMode(msg.mode + 1)
            self.set_mode(new_mode, period=msg.period, duration=msg.duration)
        except ValueError:
            self.node.get_logger().error(f"Unknown mode: {msg.mode}")
    
    def publish_value(self):
        left_value, right_value = self.logic_controller.bright()  
        
        msg = Brightness()
        msg.left_value = int(left_value) 
        msg.right_value = int(right_value)
        self.lantern_pub.publish(msg)

    def back_to_solid(self):
        self.logic_controller.set_mode(LightMode.SOLID)

    def set_mode(self, mode: LightMode, period : float, duration : float):
        self.logic_controller.set_mode(mode, period=period)
        
        if self.stop_effect_timer:
            self.node.destroy_timer(self.stop_effect_timer)
            self.stop_effect_timer = None
            
        self.stop_effect_timer = self.node.create_timer(duration, self.back_to_solid)
    

    def main_timer_callback(self):
        if self.state == 0:
            self.set_mode(LightMode.ONE_BY_ONE, period=0.5, duration=5.0)
            
        elif self.state == 1:
            self.set_mode(LightMode.BLINK, period=0.5, duration=5.0)
            
        elif self.state == 2:
            self.logic_controller.set_mode(LightMode.SOLID)
            if self.stop_effect_timer:
                self.node.destroy_timer(self.stop_effect_timer)
                self.stop_effect_timer = None

        elif self.state == 3:
            self.set_mode(LightMode.FADE, period=0.5, duration=5.0)

        self.state = (self.state + 1) % 4
        
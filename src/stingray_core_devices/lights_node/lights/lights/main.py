import rclpy
from .light_system import LanternControl

def main(args=None):
    rclpy.init(args=args)
    controller = LanternControl()
    
    try:
        rclpy.spin(controller.node)
    except KeyboardInterrupt:
        pass 
    finally:
        if controller.stop_effect_timer:
            controller.stop_effect_timer.cancel()
        if controller.publish_timer:
            controller.publish_timer.cancel()
        if controller.main_timer:
            controller.main_timer.cancel()
        
    controller.node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
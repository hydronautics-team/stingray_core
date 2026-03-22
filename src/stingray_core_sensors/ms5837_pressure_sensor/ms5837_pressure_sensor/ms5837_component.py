import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from sensor_msgs.msg import FluidPressure
from sensor_msgs.msg import Temperature
from std_msgs.msg import Float32

from rcl_interfaces.msg import SetParametersResult

from . import ms5837

import time


class BarComponentr:
    def __init__(self):
        self.sensor = ms5837.MS5837_02BA()
        
        if not self.sensor.init():
                print("Sensor could not be initialized")
                exit(1)

        if not self.sensor.read():
                print("Sensor read failed!")
                exit(1)

        print("Pressure: {} atm {} Torr {} psi".format(
                round( self.sensor.pressure(ms5837.UNITS_atm), 2),
                round( self.sensor.pressure(ms5837.UNITS_Torr), 2),
                round( self.sensor.pressure(ms5837.UNITS_psi), 2),
        ))

        print("Temperature: {} C {} F {} K".format(
                round( self.sensor.temperature(ms5837.UNITS_Centigrade), 2),
                round( self.sensor.temperature(ms5837.UNITS_Farenheit), 2),
                round( self.sensor.temperature(ms5837.UNITS_Kelvin), 2),
        ))

        self.freshwaterDepth = self.sensor.depth() # default is freshwater
        self.sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
        self.saltwaterDepth = self.sensor.depth() # No nead to read() again
        self.sensor.setFluidDensity(1000) # kg/m^3


        # TODO me
        self.ajust_depth = 0.1 # m
        self.init_fresh_depth = self.freshwaterDepth - self.ajust_depth
        self.init_salt_depth = self.saltwaterDepth - self.ajust_depth

        print("Depth: {} m (freshwater) {} m (saltwater)".format(
                round(self.init_fresh_depth , 3),
                round(self.init_salt_depth , 3),
        ))

        print("MSL Relative Altitude: {} m".format( self.sensor.altitude() )) # relative to Mean Sea Level pressure in air

        time.sleep(1)

    def pressure_value(self):
        if self.sensor.read():
                hpa_data = self.sensor.pressure()
                psi_data = self.sensor.pressure(ms5837.UNITS_psi)
        else:
                exit(1)
        return hpa_data, psi_data

    def temperature_value(self):
        if self.sensor.read():
                temp_degrees = self.sensor.temperature()
                temp_farenheit = self.sensor.temperature(ms5837.UNITS_Farenheit) 
        else:
                exit(1)
        return temp_degrees, temp_farenheit

    def depth_value(self):
        if self.sensor.read():
                depth_data = self.sensor.depth()
        else:
                exit(1)
        return depth_data

    def depth_init_error(self):
           return self.init_fresh_depth, self.init_salt_depth


class BarNode(Node):

    def __init__(self):
        super().__init__('ms5837_node')
        self.pub_pressure = self.create_publisher(Float32, 'ms5837/pressure', 10)
        self.pub_temp = self.create_publisher(Float32, 'ms5837/temperature', 10)
        self.pub_depth = self.create_publisher(Float32, 'ms5837/depth', 10)
        self.pub_odom = self.create_publisher(Odometry, 'ms5837/odom', 10)

        timer_period = 0.02  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.ms5837_data = BarComponentr()

        self.msg_pressure = Float32()
        self.msg_temp = Float32()
        self.msg_depth = Float32()
        self.msg_odom = Odometry()

        self.init_fresh, self.init_salt = self.ms5837_data.depth_init_error()

    def timer_callback(self):

        hpa_data, psi_data = self.ms5837_data.pressure_value()
        temp_degrees, temp_farenheit = self.ms5837_data.temperature_value()
        depth_data = self.ms5837_data.depth_value()

        self.msg_pressure.data = round(hpa_data, 1)

        self.msg_temp.data = round(temp_degrees, 1)

        # TODO me
        ajust_depth = 0.1 # meter

        self.msg_depth.data = round(depth_data - ajust_depth - self.init_fresh, 3)
        depth_data_mm = round(self.msg_depth.data*1000, 3)

        self.msg_odom.header.stamp = self.get_clock().now().to_msg()
        self.msg_odom.header.frame_id = "ms5837_link"
        self.msg_odom.pose.pose.position.z = - self.msg_depth.data

        self.pub_pressure.publish(self.msg_pressure)
        self.pub_temp.publish(self.msg_temp)
        self.pub_depth.publish(self.msg_depth)
        self.pub_odom.publish(self.msg_odom)

def main(args=None):
    rclpy.init(args=args)
    ms5837_node = BarNode()

    rclpy.spin(ms5837_node)

    ms5837_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

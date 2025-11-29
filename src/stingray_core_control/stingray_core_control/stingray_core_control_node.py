#!/usr/bin/env python3
"""
Stingray Core Control Node
Minimal ROS2 node skeleton with 100 Hz control loop callback.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rclpy.parameter import SetParametersResult

import time
import math

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, UInt8, Bool, UInt8MultiArray

from .thruster_mixer import ThrusterMixer


class StingrayCoreControlNode(Node):
    def __init__(self):
        super().__init__('stingray_core_control_node')

        # --- параметры ---
        self.declare_parameter('rate_hz', 100.0)
        self.rate_hz = float(self.get_parameter(
            'rate_hz').get_parameter_value().double_value)

        self.declare_parameter('topic_imu_angular', '/vectornav/angular')
        self.declare_parameter('topic_imu_linear_accel',
                               '/vectornav/imu_accel')
        self.declare_parameter('topic_imu_angular_rate', '/vectornav/imu_rate')
        self.declare_parameter('topic_loop_flags', '/control/loop_flags')
        self.declare_parameter('topic_pressure_sensor', '/sensors/pressure')
        self.declare_parameter('topic_control_data', '/control/data')
        self.declare_parameter('vectornav_yaw_offset_deg', 0.0)

        self.topic_imu_angular = self.get_parameter(
            'topic_imu_angular').get_parameter_value().string_value
        self.topic_imu_linear_accel = self.get_parameter(
            'topic_imu_linear_accel').get_parameter_value().string_value
        self.topic_imu_angular_rate = self.get_parameter(
            'topic_imu_angular_rate').get_parameter_value().string_value
        self.topic_loop_flags = self.get_parameter(
            'topic_loop_flags').get_parameter_value().string_value
        self.topic_pressure_sensor = self.get_parameter(
            'topic_pressure_sensor').get_parameter_value().string_value
        self.topic_control_data = self.get_parameter(
            'topic_control_data').get_parameter_value().string_value
        self.vectornav_yaw_offset = float(self.get_parameter(
            'vectornav_yaw_offset_deg').get_parameter_value().double_value)
        
        self.declare_parameter('thrusters', ['top_front_left', 'top_front_right', 'top_rear_left', 'top_rear_right',
        'bottom_front_left', 'bottom_front_right', 'bottom_rear_left', 'bottom_rear_right',
        'middle_left', 'middle_right'])
        self.declare_parameter('axes', ["u_surge", "u_sway", "u_heave", "u_roll", "u_pitch", "u_yaw"])
        self.thrusters = list(self.get_parameter('thrusters').get_parameter_value().string_array_value)
        self.axes = list(self.get_parameter('axes').get_parameter_value().string_array_value)

        coeffs = {}
        for t in self.thrusters:
            row = []
            for a in self.axes:
                param_name = f"{t}_{a}"
                self.declare_parameter(param_name, 0)
                p = self.get_parameter(param_name).get_parameter_value()
                val = p.integer_value
                row.append(val)
            coeffs[t] = row

        # create pure mixer
        self.mixer = ThrusterMixer(self.thrusters, self.axes, coeffs)
        self.get_logger().info(f"ThrusterMixer initialized: thrusters={self.thrusters}, axes={self.axes}, coeffs= {coeffs}")

        # subscribe to parameter changes to update coeffs dynamically
        self.add_on_set_parameters_callback(self._on_params_changed)

        qos = QoSProfile(depth=10)

        # --- подписки ---
        self.sub_imu_angular = self.create_subscription(
            Vector3, self.topic_imu_angular, self.imu_angular_callback, qos)
        self.sub_imu_linear_accel = self.create_subscription(
            Vector3, self.topic_imu_linear_accel, self.imu_linear_accel_callback, qos)
        self.sub_imu_angular_rate = self.create_subscription(
            Vector3, self.topic_imu_angular_rate, self.imu_angular_rate_callback, qos)

        # loop flags — используем UInt8 (битфлаги)
        self.sub_control_mode_flags = self.create_subscription(
            UInt8, self.topic_loop_flags, self.control_mode_flags_callback, qos)

        # pressure: обычно Float32
        self.sub_pressure_sensor = self.create_subscription(
            Float32, self.topic_pressure_sensor, self.pressure_sensor_callback, qos)

        # control data (входные "impact" команды)
        self.sub_control_data = self.create_subscription(
            Twist, self.topic_control_data, self.control_data_callback, qos)

        # паблишер
        self.pub_thruster_cmd = self.create_publisher(
            UInt8MultiArray, '/thruster/cmd', qos)

        # --- инициализация состояний ---
        self.imu_yaw = 0.0
        self.imu_pitch = 0.0
        self.imu_roll = 0.0
        self.imu_accel_x = 0.0
        self.imu_accel_y = 0.0
        self.imu_accel_z = 0.0
        self.imu_rate_x = 0.0
        self.imu_rate_y = 0.0
        self.imu_rate_z = 0.0

        # входные импакты (по умолчанию нули)
        self.impact_surge = 0.0
        self.impact_sway = 0.0
        self.impact_depth = 0.0
        self.impact_roll = 0.0
        self.impact_pitch = 0.0
        self.impact_yaw = 0.0

        # флаги управления (по умолчанию off)
        self.control_mode_flag_surge = False
        self.control_mode_flag_sway = False
        self.control_mode_flag_heave = False
        self.control_mode_flag_yaw = False
        self.control_mode_flag_pitch = False
        self.control_mode_flag_roll = False

        self.depth = 0.0

        # таймер
        self.last_time = time.time()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

        self._last_status_log = 0.0
        self._status_log_interval = 1.0  # сек

        self.get_logger().info(
            f"StingrayCoreControlNode started at {self.rate_hz} Hz")

    def normalize_angle_deg(self, angle_deg):
        """Нормализует угол в градусах в диапазон [-180, 180)."""
        a = (angle_deg + 180.0) % 360.0 - 180.0
        return a

    def control_loop(self):
        now = time.time()
        dt = now - self.last_time if self.last_time is not None else 0.0
        self.last_time = now

        # === 1. Определяем управляющие воздействия ===
        if self.control_mode_flag_surge:
            u_surge = self.compute_surge()
        else:
            u_surge = self.impact_surge

        if self.control_mode_flag_sway:
            u_sway = self.compute_sway()
        else:
            u_sway = self.impact_sway

        if self.control_mode_flag_heave:
            u_heave = self.compute_heave()
        else:
            u_heave = self.impact_depth

        if self.control_mode_flag_roll:
            u_roll = self.compute_roll()
        else:
            u_roll = self.impact_roll

        if self.control_mode_flag_pitch:
            u_pitch = self.compute_pitch()
        else:
            u_pitch = self.impact_pitch

        if self.control_mode_flag_yaw:
            u_yaw = self.compute_yaw()
        else:
            u_yaw = self.impact_yaw

        # === 2. Преобразуем в команды thrusters ===
        control_list = [u_surge, u_sway, u_heave, u_roll, u_pitch, u_yaw]
        thruster_cmds = self.mixer.mix_from_list(control_list)

        # === 3. Публикуем UInt8MultiArray ===
        msg = UInt8MultiArray()
        msg.data = thruster_cmds
        self.pub_thruster_cmd.publish(msg)

        # Логируем статус 1 раз в секунду, а не каждый тик
        if now - self._last_status_log > self._status_log_interval:
            # self.get_logger().info(f"Thrusters command: {thruster_cmds}")
            # self.get_logger().debug(f"Control loop tick: dt={dt:.6f} s")
            self._last_status_log = now

    # --- Колбэки ---
    def imu_angular_callback(self, msg: Vector3):
        try:
            self.imu_yaw = self.normalize_angle_deg(
                float(msg.x) + self.vectornav_yaw_offset)
            self.imu_pitch = float(msg.y)
            self.imu_roll = float(msg.z)
        except Exception as e:
            self.get_logger().warning(f"Error parsing imu_angular msg: {e}")

    def imu_linear_accel_callback(self, msg: Vector3):
        try:
            self.imu_accel_x = float(msg.x)
            self.imu_accel_y = float(msg.y)
            self.imu_accel_z = float(msg.z)
        except Exception as e:
            self.get_logger().warning(
                f"Error parsing imu_linear_accel msg: {e}")

    def imu_angular_rate_callback(self, msg: Vector3):
        try:
            self.imu_rate_x = float(msg.x)
            self.imu_rate_y = float(msg.y)
            self.imu_rate_z = float(msg.z)
        except Exception as e:
            self.get_logger().warning(
                f"Error parsing imu_angular_rate msg: {e}")

    def pressure_sensor_callback(self, msg: Float32):
        try:
            self.depth = float(msg.data)
        except Exception as e:
            self.get_logger().warning(f"Error parsing depth msg: {e}")

    def control_data_callback(self, msg: Twist):
        self.get_logger().info(f"control_data_callback: {msg}")

        # impact команды — просто копируем вход
        self.impact_surge = float(msg.linear.x)
        self.impact_sway = float(msg.linear.y)
        self.impact_depth = float(msg.linear.z)
        self.impact_roll = float(msg.angular.x)
        self.impact_pitch = float(msg.angular.y)
        self.impact_yaw = float(msg.angular.z)

    def control_mode_flags_callback(self, msg: UInt8):
        flags = int(msg.data)
        self.control_mode_flag_surge = bool(flags & (1 << 0))
        self.control_mode_flag_sway = bool(flags & (1 << 1))
        self.control_mode_flag_heave = bool(flags & (1 << 2))
        self.control_mode_flag_yaw = bool(flags & (1 << 3))
        self.control_mode_flag_pitch = bool(flags & (1 << 4))
        self.control_mode_flag_roll = bool(flags & (1 << 5))
        self.get_logger().debug(f"Loop flags updated: {flags:02x}")

    # === Заглушки регуляторов ===
    def compute_surge(self): return 0.0
    def compute_sway(self): return 0.0
    def compute_heave(self): return 0.0
    def compute_roll(self): return 0.0
    def compute_pitch(self): return 0.0
    def compute_yaw(self): return 0.0

    def _on_params_changed(self, params):
        """
        Called when parameters are set via rqt/ros2 param.
        Update coefficients that match <thruster>_<axis>.
        """
        updated = {}
        for p in params:
            name = p.name
            # match param names like 'top_front_left_u_surge'
            for t in self.thrusters:
                if name.startswith(t + "_"):
                    # extract axis part just to sanity-check length later
                    val = None
                    if p.type_ == Parameter.Type.INTEGER:
                        val = p.value
                    elif p.type_ == Parameter.Type.DOUBLE:
                        val = p.value
                    else:
                        try:
                            val = float(p.value)
                        except Exception:
                            continue
                    # accumulate into updated dict
                    if t not in updated:
                        updated[t] = list(self.mixer.coeffs[t])  # copy existing row
                    # find axis index
                    axis = name[len(t)+1:]
                    if axis in self.axes:
                        idx = self.axes.index(axis)
                        updated[t][idx] = val
        if updated:
            self.mixer.update_coeffs(updated)
            self.get_logger().info(f"Updated mixer coeffs for: {list(updated.keys())}")
        return SetParametersResult(successful=True)

def main(args=None):
    rclpy.init(args=args)
    node = StingrayCoreControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

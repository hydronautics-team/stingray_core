#!/usr/bin/env python3
"""
Stingray Core Control Node
Minimal ROS2 node skeleton with 100 Hz control loop callback.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult

import time

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64, UInt8, Bool, UInt8MultiArray
from sensor_msgs.msg import Imu
from vectornav_msgs.msg import CommonGroup
from dvl_msgs.msg import DVL

from .control.thruster_mixer import ThrusterMixer
from .control.controllers import (
    YawController, PitchController, RollController,
    DepthController, SurgeController, SwayController
)
from .utils.save_params import save_params
from .state.imu import ImuState
from .state.control import ControlState
from .control.axis_controller import (
    AxisController,
    AngularAxisController,
    LinearAxisController,
    LinearVelocityAxisController,
)


class StingrayCoreControlNode(Node):
    def __init__(self):
        super().__init__('stingray_core_control_node')
        self._init_config()
        self._init_control_core()
        self._init_ros_interfaces()
        self._init_runtime()

        self.get_logger().info(
            f"Started at {self.rate_hz} Hz | axes={self.axes} | thrusters={len(self.thrusters)}"
        )

    def control_loop(self):
        now = time.time()
        dt = now - self.last_time if self.last_time is not None else 0.0
        self.last_time = now
        self.last_dt = max(min(dt, 0.05), 1e-3)

        self._update_motion_estimation(self.last_dt)

        # === 1. Определяем управляющие воздействия ===
        u: dict[str, float] = {}

        for axis in self.axes:
            if axis in self.axis_ctrl and self.control.enabled[axis]:
                u[axis] = self.axis_ctrl[axis].compute(
                    target=self.control_setpoint[axis],
                    dt=self.last_dt,
                )
            else:
                # Для разомкнутого контура воздействие одноразовое: 1 цикл,
                # затем сбрасывается до следующей новой команды.
                if self.open_loop_pending[axis]:
                    u[axis] = self.control_input[axis]
                    self.open_loop_pending[axis] = False
                else:
                    u[axis] = 0.0

            # Оставляем последнее фактически применённое воздействие в state
            self.control.impact[axis] = u[axis]

        # === 2. Преобразуем в команды thrusters ===
        control_list = [u[a] for a in self.axes]
        thruster_cmds = self.mixer.mix_from_list(control_list)

        # === 3. Публикуем thruster_cmd ===
        msg = UInt8MultiArray()
        msg.data = thruster_cmds
        self.pub_thruster_cmd.publish(msg)

        # === 4. Публикуем ориентацию ===
        self.pub_yaw.publish(Float64(data=self.imu.yaw))
        self.pub_pitch.publish(Float64(data=self.imu.pitch))
        self.pub_roll.publish(Float64(data=self.imu.roll))
        self.pub_depth.publish(Float64(data=self.depth))


    def _init_config(self):
        self.declare_parameter('rate_hz', 100.0)
        self.rate_hz = float(self.get_parameter('rate_hz').value)

        self.declare_parameter('flag_setup_feedback_speed', False)
        self.flag_setup_feedback_speed = bool(self.get_parameter('flag_setup_feedback_speed').value)

        self.declare_parameter('debug_publish', False)

        defaults = {
            'topic_imu_angular': '/vectornav/raw/common',
            'topic_imu_linear_accel': '/vectornav/imu',
            'topic_imu_angular_rate': '/vectornav/imu',
            'topic_dvl_data': '/dvl/data',
            'topic_loop_flags': '/control/loop_flags',
            'topic_pressure_sensor': '/stingray_core/pressure_sensor/depth',
            'topic_control_data': '/control/data',
            'topic_zero_yaw': '/imu/zero_yaw',
        }

        for name, default in defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        self.declare_parameter('axes', ["surge", "sway", "heave", "roll", "pitch", "yaw"])
        self.axes = list(self.get_parameter('axes').value)

        self.declare_parameter('use_dvl_velocity', False)
        self.use_dvl_velocity = bool(self.get_parameter('use_dvl_velocity').value)

        self.declare_parameter('use_distance_measurement', False)
        self.use_distance_measurement = bool(self.get_parameter('use_distance_measurement').value)

        self.declare_parameter('dvl_velocity_alpha', 0.2)
        self.dvl_velocity_alpha = float(self.get_parameter('dvl_velocity_alpha').value)

        self.declare_parameter('dvl_timeout_sec', 0.5)
        self.dvl_timeout_sec = float(self.get_parameter('dvl_timeout_sec').value)

        self.declare_parameter('thrusters', Parameter.Type.STRING_ARRAY)
        self.thrusters = list(self.get_parameter('thrusters').value)

    def _init_control_core(self):
        coeffs = {}
        for thr in self.thrusters:
            row = []
            for axis in self.axes:
                pname = f"{thr}_{axis}"
                if not self.has_parameter(pname):
                    self.declare_parameter(
                        pname,
                        0.0,
                        ParameterDescriptor(dynamic_typing=True),
                    )
                row.append(float(self.get_parameter(pname).value))
            coeffs[thr] = row

        self.mixer = ThrusterMixer(self.thrusters, self.axes, coeffs)

        axis_class_map = {
            'yaw': YawController,
            'pitch': PitchController,
            'roll': RollController,
            'heave': DepthController,
            'surge': SurgeController,
            'sway': SwayController,
        }

        self.controllers = {}
        self.param_keys = ["K_p", "K_1", "K_2", "K_i", "I_min", "I_max", "out_sat", "ap_K", "ap_T"]
        for axis in self.axes:
            params = {}
            for key in self.param_keys:
                pname = f"controllers.{axis}.{key}"
                self.declare_parameter(
                    pname,
                    0.0,
                    ParameterDescriptor(dynamic_typing=True),
                )
                params[key] = float(self.get_parameter(pname).value)

            if axis in ("pitch", "roll"):
                for key in ("grav_bias", "grav_gain", "grav_offset_deg"):
                    pname = f"controllers.{axis}.{key}"
                    self.declare_parameter(
                        pname,
                        0.0,
                        ParameterDescriptor(dynamic_typing=True),
                    )
                    params[key] = float(self.get_parameter(pname).value)

            self.controllers[axis] = axis_class_map[axis](**params)

        for ctrl in self.controllers.values():
            ctrl.set_debug_hook(self.debug_cb)

        self.imu = ImuState()
        self.control = ControlState.from_axes(self.axes)

        # Последняя полученная команда из /control/data
        self.control_input: dict[str, float] = {axis: 0.0 for axis in self.axes}
        # Удерживаемые цели для замкнутых контуров
        self.control_setpoint: dict[str, float] = {axis: 0.0 for axis in self.axes}
        # Флаг одноразовой подачи для разомкнутых контуров
        self.open_loop_pending: dict[str, bool] = {axis: False for axis in self.axes}

        self.depth = 0.0
        self.distance_to_bottom = 0.0

        self.surge_velocity_imu = 0.0
        self.sway_velocity_imu = 0.0
        self.heave_velocity_imu = 0.0
        self.surge_velocity_est = 0.0
        self.sway_velocity_est = 0.0
        self.heave_velocity_est = 0.0

        self.dvl_velocity_x = 0.0
        self.dvl_velocity_y = 0.0
        self.dvl_velocity_z = 0.0
        self.dvl_velocity_valid = False
        self.dvl_last_time = 0.0

        self.yaw_zero_offset = 0.0
        self.imu_yaw_raw = 0.0

        self.axis_ctrl: dict[str, AxisController] = {
            "yaw": AngularAxisController(
                controller=self.controllers["yaw"],
                angle_fn=lambda: self.imu.yaw,
                rate_fn=lambda: self.imu.rate_z,
                feedback_flag_fn=lambda: self.flag_setup_feedback_speed,
            ),
            "pitch": AngularAxisController(
                controller=self.controllers["pitch"],
                angle_fn=lambda: self.imu.pitch,
                rate_fn=lambda: self.imu.rate_y,
                feedback_flag_fn=lambda: self.flag_setup_feedback_speed,
            ),
            "roll": AngularAxisController(
                controller=self.controllers["roll"],
                angle_fn=lambda: self.imu.roll,
                rate_fn=lambda: self.imu.rate_x,
                feedback_flag_fn=lambda: self.flag_setup_feedback_speed,
            ),
            "heave": LinearAxisController(
                controller=self.controllers["heave"],
                pos_fn=lambda: self.depth,
                vel_fn=self._get_heave_velocity_measurement,
                accel_fn=lambda: self.imu.accel_z,
                feedback_flag_fn=lambda: self.flag_setup_feedback_speed,
            ),
            "surge": LinearVelocityAxisController(
                controller=self.controllers["surge"],
                vel_fn=self._get_surge_velocity_measurement,
                accel_fn=lambda: self.imu.accel_x,
            ),
            "sway": LinearVelocityAxisController(
                controller=self.controllers["sway"],
                vel_fn=self._get_sway_velocity_measurement,
                accel_fn=lambda: self.imu.accel_y,
            ),
        }

        # fallback для остальных осей
        # for axis in self.axes:
        #     if axis not in self.axis_ctrl:
        #         self.axis_ctrl[axis] = PassthroughAxisController()


    def _init_ros_interfaces(self):
        self._init_subscriptions()
        self._init_publishers()
        self.add_on_set_parameters_callback(self._on_params_changed)

    def _init_runtime(self):
        self.last_time = time.time()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)

    def normalize_angle_deg(self, angle_deg):
        """Нормализует угол в градусах в диапазон [-180, 180)."""
        a = (angle_deg + 180.0) % 360.0 - 180.0
        return a

    def _is_dvl_fresh(self) -> bool:
        if self.dvl_last_time <= 0.0:
            return False
        return (time.time() - self.dvl_last_time) <= self.dvl_timeout_sec

    def _update_motion_estimation(self, dt: float):
        # IMU-only оценка скоростей интегрированием ускорений
        self.surge_velocity_imu += self.imu.accel_x * dt
        self.sway_velocity_imu += self.imu.accel_y * dt
        self.heave_velocity_imu += self.imu.accel_z * dt

        use_dvl_now = self.use_dvl_velocity and self.dvl_velocity_valid and self._is_dvl_fresh()
        alpha = max(0.0, min(1.0, self.dvl_velocity_alpha))

        if use_dvl_now:
            # IMU + DVL: blended correction для снижения дрейфа IMU
            self.surge_velocity_est = alpha * self.surge_velocity_imu + (1.0 - alpha) * self.dvl_velocity_x
            self.sway_velocity_est = alpha * self.sway_velocity_imu + (1.0 - alpha) * self.dvl_velocity_y
            self.heave_velocity_est = alpha * self.heave_velocity_imu + (1.0 - alpha) * self.dvl_velocity_z
        else:
            # IMU-only
            self.surge_velocity_est = self.surge_velocity_imu
            self.sway_velocity_est = self.sway_velocity_imu
            self.heave_velocity_est = self.heave_velocity_imu

    def _get_surge_velocity_measurement(self) -> float:
        return self.surge_velocity_est

    def _get_sway_velocity_measurement(self) -> float:
        return self.sway_velocity_est

    def _get_heave_velocity_measurement(self) -> float:
        return self.heave_velocity_est

    def _init_subscriptions(self):
        # Explicit QoS profiles for deterministic behavior by topic class.
        qos_sensor = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        qos_command = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        qos_event = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_imu_angular = self.create_subscription(
            CommonGroup, self.topic_imu_angular,
            self.imu_angular_callback, qos_sensor
        )

        self.sub_imu_linear_accel = self.create_subscription(
            Imu, self.topic_imu_linear_accel,
            self.imu_linear_accel_callback, qos_sensor
        )

        self.sub_imu_angular_rate = self.create_subscription(
            Imu, self.topic_imu_angular_rate,
            self.imu_angular_rate_callback, qos_sensor
        )

        self.sub_dvl_data = self.create_subscription(
            DVL, self.topic_dvl_data,
            self.dvl_data_callback, qos_sensor
        )

        self.sub_control_mode_flags = self.create_subscription(
            UInt8, self.topic_loop_flags,
            self.control_mode_flags_callback, qos_command
        )

        self.sub_pressure_sensor = self.create_subscription(
            Float64, self.topic_pressure_sensor,
            self.pressure_sensor_callback, qos_sensor
        )

        self.sub_control_data = self.create_subscription(
            Twist, self.topic_control_data,
            self.control_data_callback, qos_command
        )

        self.sub_zero_yaw = self.create_subscription(
            Bool, self.topic_zero_yaw,
            self.zero_yaw_callback, qos_event
        )

    def _init_publishers(self):
        qos_actuation = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        qos_telemetry = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        qos_debug = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_thruster_cmd = self.create_publisher(
            UInt8MultiArray, '/thruster/cmd', qos_actuation
        )

        self.pub_yaw = self.create_publisher(Float64, '~/orientation/yaw', qos_telemetry)
        self.pub_pitch = self.create_publisher(Float64, '~/orientation/pitch', qos_telemetry)
        self.pub_roll = self.create_publisher(Float64, '~/orientation/roll', qos_telemetry)
        self.pub_depth = self.create_publisher(Float64, '~/orientation/depth', qos_telemetry)
        

        self.pub_err_position = self.create_publisher(Float64, "~/debug/err_position", qos_debug)
        self.pub_output_pi = self.create_publisher(Float64, "~/debug/output_pi", qos_debug)
        self.pub_feedback_speed = self.create_publisher(Float64, "~/debug/feedback_speed", qos_debug)
        self.pub_measurement_rate = self.create_publisher(Float64, "~/debug/measurement_rate", qos_debug)
        self.pub_out = self.create_publisher(Float64, "~/debug/out", qos_debug)
    
    # --- Колбэки ---
    def imu_angular_callback(self, msg: CommonGroup):
        try:
            ypr: Vector3 = msg.yawpitchroll

            self.imu_yaw_raw = float(ypr.x)

            self.imu.yaw = self.normalize_angle_deg(
            self.imu_yaw_raw - self.yaw_zero_offset
            )
            self.imu.pitch = float(ypr.y)
            self.imu.roll = float(ypr.z)

        except Exception as e:
            self.get_logger().warning(
                f"Error parsing CommonGroup yawpitchroll: {e}"
            )

    def imu_angular_rate_callback(self, msg: Imu):
        try:
            self.imu.rate_x = float(msg.angular_velocity.x)
            self.imu.rate_y = float(msg.angular_velocity.y)
            self.imu.rate_z = float(msg.angular_velocity.z)
        except Exception as e:
            self.get_logger().warning(
                f"Error parsing imu angular_velocity from Imu msg: {e}"
            )

    def imu_linear_accel_callback(self, msg: Imu):
        try:
            self.imu.accel_x = float(msg.linear_acceleration.x)
            self.imu.accel_y = float(msg.linear_acceleration.y)
            self.imu.accel_z = float(msg.linear_acceleration.z)
        except Exception as e:
            self.get_logger().warning(
                f"Error parsing imu linear acceleration from Imu msg: {e}"
            )

    def dvl_data_callback(self, msg: DVL):
        try:
            self.dvl_velocity_x = float(msg.velocity.x)
            self.dvl_velocity_y = float(msg.velocity.y)
            self.dvl_velocity_z = float(msg.velocity.z)
            self.dvl_velocity_valid = bool(msg.velocity_valid)
            self.dvl_last_time = time.time()

            if self.use_distance_measurement:
                self.distance_to_bottom = float(msg.altitude)
        except Exception as e:
            self.get_logger().warning(f"Error parsing DVL msg: {e}")
    
    def zero_yaw_callback(self, msg: Bool):
        if not msg.data:
            return

        self.yaw_zero_offset = self.imu_yaw_raw
        self.get_logger().info(
            f"Yaw zeroed at {self.yaw_zero_offset:.2f} deg")

    def pressure_sensor_callback(self, msg: Float64):
        try:
            self.depth = float(msg.data)
        except Exception as e:
            self.get_logger().warning(f"Error parsing depth msg: {e}")

    def control_data_callback(self, msg: Twist):
        incoming = {
            "surge": float(msg.linear.x),
            "sway": float(msg.linear.y),
            "heave": float(msg.linear.z),
            "roll": float(msg.angular.x),
            "pitch": float(msg.angular.y),
            "yaw": float(msg.angular.z),
        }

        for axis, value in incoming.items():
            self.control_input[axis] = value

            if self.control.enabled.get(axis, False):
                # Замкнутый контур: держим setpoint до новой команды.
                self.control_setpoint[axis] = value
            else:
                # Разомкнутый контур: дать команду только на один цикл.
                self.open_loop_pending[axis] = True

    def control_mode_flags_callback(self, msg: UInt8):
        flags = int(msg.data)
        new_enabled = {
            "surge": bool(flags & (1 << 0)),
            "sway":  bool(flags & (1 << 1)),
            "heave": bool(flags & (1 << 2)),
            "yaw":   bool(flags & (1 << 3)),
            "pitch": bool(flags & (1 << 4)),
            "roll":  bool(flags & (1 << 5)),
        }

        changed = []
        for axis, new_value in new_enabled.items():
            old_value = bool(self.control.enabled.get(axis, False))
            if old_value != new_value:
                changed.append(f"{axis}: {old_value} -> {new_value}")

                if new_value:
                    # При включении контура берём последнюю принятую команду как цель.
                    self.control_setpoint[axis] = self.control_input[axis]
                else:
                    # При выключении контура не повторяем старую команду.
                    self.open_loop_pending[axis] = False

            self.control.enabled[axis] = new_value

        if changed:
            self.get_logger().info(
                f"Control flags changed (0x{flags:02X}): " + ", ".join(changed)
            )

    def _on_params_changed(self, params: list[Parameter]) -> SetParametersResult:
        try:
            thruster_params: list[Parameter] = []
            controller_params: list[Parameter] = []
            node_params: list[Parameter] = []

            # --- 1. Классификация параметров ---
            for p in params:
                if self._is_thruster_param(p.name):
                    thruster_params.append(p)
                elif self._is_controller_param(p.name):
                    controller_params.append(p)
                else:
                    node_params.append(p)

            # --- 2. Применяем controller params ---
            for p in controller_params:
                _, axis, key = p.name.split(".", 2)

                ctrl = self.controllers.get(axis)
                if ctrl and hasattr(ctrl, key):
                    setattr(ctrl, key, p.value)
                    self.get_logger().info(
                        f"Controller '{axis}': {key} = {p.value}"
                    )
                else:
                    self.get_logger().warning(
                        f"Ignored controller param: {p.name}"
                    )

            # --- 3. Применяем node params ---
            for p in node_params:
                if hasattr(self, p.name):
                    setattr(self, p.name, p.value)
                    self.get_logger().info(
                        f"Node param updated: {p.name} = {p.value}"
                    )

            # --- 4. Сохраняем параметры ---
            if thruster_params:
                save_params(self, thruster_params, "thruster_matrix")

            if controller_params:
                save_params(self, controller_params, "controllers")

            if node_params:
                save_params(self, node_params, "stingray_core_control_node")

            # --- 5. Обновляем mixer ---
            if thruster_params:
                self._update_mixer_from_params(thruster_params)

            return SetParametersResult(successful=True)

        except Exception as e:
            self.get_logger().error(f"_on_params_changed failed: {e}")
            return SetParametersResult(successful=False, reason=str(e))

    def _update_mixer_from_params(self, params: list[Parameter]):
        coeffs_update = {}

        for p in params:
            for thr in self.thrusters:
                if not p.name.startswith(f"{thr}_"):
                    continue

                axis = p.name[len(thr) + 1 :]
                if axis not in self.axes:
                    continue

                idx = self.axes.index(axis)
                row = list(self.mixer.coeffs.get(thr, [0.0] * len(self.axes)))
                row[idx] = float(p.value)
                coeffs_update[thr] = row

        if coeffs_update:
            self.mixer.update_coeffs(coeffs_update)
            self.get_logger().info(
                f"Updated thruster coeffs: {coeffs_update}"
            )

    def _is_thruster_param(self, name: str) -> bool:
        return any(
            name.startswith(f"{thr}_") and name[len(thr)+1:] in self.axes
            for thr in self.thrusters
        )

    def _is_controller_param(self, name: str) -> bool:
        return name.startswith("controllers.")
        
    def debug_cb(self, data: dict):
        if not self.get_parameter("debug_publish").value:
            return

        self.pub_err_position.publish(Float64(data=data["err_position"]))
        self.pub_output_pi.publish(Float64(data=data["output_pi"]))
        self.pub_feedback_speed.publish(Float64(data=data["feedback_speed"]))
        self.pub_measurement_rate.publish(Float64(data=data["measurement_rate"]))
        self.pub_out.publish(Float64(data=data["out"]))




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

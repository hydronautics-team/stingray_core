#!/usr/bin/env python3
"""
Stingray Core Control Node
Minimal ROS2 node skeleton with 100 Hz control loop callback.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

import time

from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float32, Float64, UInt8, Bool, UInt8MultiArray
from sensor_msgs.msg import Imu
from vectornav_msgs.msg import CommonGroup
from rclpy.qos import qos_profile_sensor_data

from .utils.thruster_mixer import ThrusterMixer
from .utils.controllers import (
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
)


class StingrayCoreControlNode(Node):
    def __init__(self):
        super().__init__('stingray_core_control_node')

        # --- параметры ---
        self.declare_parameter('rate_hz', 100.0)
        self.rate_hz = float(self.get_parameter(
            'rate_hz').get_parameter_value().double_value)
        
        self.declare_parameter('flag_setup_feedback_speed', False)
        self.flag_setup_feedback_speed = self.get_parameter(
            'flag_setup_feedback_speed').get_parameter_value().bool_value

        self.declare_parameter('topic_imu_angular', '/vectornav/raw/common')
        self.declare_parameter('topic_imu_linear_accel',
                               '/vectornav/imu_accel')
        self.declare_parameter('topic_imu_angular_rate', '/vectornav/imu')
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
        
        # --- yaw zeroing ---
        self.yaw_zero_offset = 0.0
        self.imu_yaw_raw = 0.0

        self.declare_parameter('topic_zero_yaw', '/imu/zero_yaw')
        self.topic_zero_yaw = self.get_parameter(
            'topic_zero_yaw').get_parameter_value().string_value

        self.sub_zero_yaw = self.create_subscription(
            Bool, self.topic_zero_yaw, self.zero_yaw_callback, 10)

        
        if not self.has_parameter('thrusters'):
            self.declare_parameter('thrusters', ["",""])

        self.declare_parameter('axes_u', ["u_surge", "u_sway", "u_heave", "u_roll", "u_pitch", "u_yaw"])
        self.thrusters = list(self.get_parameter('thrusters').get_parameter_value().string_array_value)
        self.axes_u = list(self.get_parameter('axes_u').get_parameter_value().string_array_value)
        

        coeffs = {}
        for t in self.thrusters:
            row = []
            for a in self.axes_u:
                param_name = f"{t}_{a}"
                if not self.has_parameter(param_name):
                    self.declare_parameter(param_name, 0)
                p = self.get_parameter(param_name).get_parameter_value()
                val = p.integer_value
                row.append(val)
            coeffs[t] = row

        # create pure mixer
        self.mixer = ThrusterMixer(self.thrusters, self.axes_u, coeffs)
        self.get_logger().info(f"ThrusterMixer initialized: thrusters={self.thrusters}, axes={self.axes_u}, coeffs= {coeffs}")

        # subscribe to parameter changes to update coeffs dynamically

        self.controllers = {}

        axis_class_map = {
            'yaw': YawController,
            'pitch': PitchController,
            'roll': RollController,
            'heave': DepthController,
            'surge': SurgeController,
            'sway': SwayController
        }

        self.declare_parameter('axes', ["surge", "sway", "heave", "roll", "pitch", "yaw"])
        self.axes = list(self.get_parameter('axes').get_parameter_value().string_array_value)

        self.param_keys = ["K_p", "K_1", "K_2", "K_i", "I_min", "I_max", "out_sat", "ap_K", "ap_T"]

        # внутри ноды
        for axis in self.axes:
            # Получаем список ключей для оси
            self.declare_parameter(axis, ["K_p", "K_1", "K_2", "K_i", "I_min", "I_max", "out_sat", "ap_K", "ap_T"])

            # Загружаем параметры
            params_dict = {}
            for key in self.param_keys:
                param_name = f'controllers.{axis}.{key}'
                self.declare_parameter(param_name, 0.0)
                params_dict[key] = self.get_parameter(param_name).value

            # Инициализируем контроллер конкретного класса
            ctrl_class = axis_class_map[axis]  # всегда берём нужный класс
            self.controllers[axis] = ctrl_class(**params_dict)


        self.get_logger().info(f"Controllers initialized: {self.controllers}")

        self.imu = ImuState()
        self.control = ControlState.from_axes(self.axes)

        # --- Axis controllers (angular / linear split) ---
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
            "heave": LinearAxisController(
                controller=self.controllers["heave"],
                pos_fn=lambda: self.depth,
                vel_fn=lambda: 0.0,   # пока нет оценки вертикальной скорости
                accel_fn=lambda: 0.0,
                feedback_flag_fn=lambda: self.flag_setup_feedback_speed,
            ),
        }

        qos = QoSProfile(depth=10)

        # --- подписки ---
        self.sub_imu_angular = self.create_subscription(
            CommonGroup, "/vectornav/raw/common", self.imu_angular_callback, qos_profile_sensor_data)
        self.sub_imu_linear_accel = self.create_subscription(
            Vector3, self.topic_imu_linear_accel, self.imu_linear_accel_callback, qos)
        self.sub_imu_angular_rate = self.create_subscription(
            Imu, self.topic_imu_angular_rate, self.imu_angular_rate_callback, qos)
        
        # --- my orientation publishers ---
        self.pub_yaw   = self.create_publisher(Float64, '~/orientation/yaw', 10)
        self.pub_pitch = self.create_publisher(Float64, '~/orientation/pitch', 10)
        self.pub_roll  = self.create_publisher(Float64, '~/orientation/roll', 10)


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

        self.depth = 0.0
        self.yaw_ctrl = self.controllers["yaw"]
        self.depth_ctrl = self.controllers["heave"]
        self.pitch_ctrl = self.controllers["pitch"]

        self.declare_parameter('debug_publish', False)
        self.pub_err_position   = self.create_publisher(Float64, "~/debug/err_position", 10)
        self.pub_output_pi      = self.create_publisher(Float64, "~/debug/output_pi", 10)
        self.pub_feedback_speed = self.create_publisher(Float64, "~/debug/feedback_speed", 10)
        self.pub_measurement_rate    = self.create_publisher(Float64, "~/debug/measurement_rate", 10)
        self.pub_out            = self.create_publisher(Float64, "~/debug/out", 10)
        self.yaw_ctrl.set_debug_hook(self.debug_cb)
        self.depth_ctrl.set_debug_hook(self.debug_cb)


        self.add_on_set_parameters_callback(self._on_params_changed)

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
        self.last_dt = dt

        # === 1. Определяем управляющие воздействия ===
        u: dict[str, float] = {}

        for axis in self.axes:
            if axis in self.axis_ctrl and self.control.enabled[axis]:
                u[axis] = self.axis_ctrl[axis].compute(
                    target=self.control.impact[axis],
                    dt=self.last_dt,
                )
            else:
                u[axis] = self.control.impact[axis]

        # === 2. Преобразуем в команды thrusters ===
        control_list = [u["surge"], u["sway"], u["heave"], u["roll"], u["pitch"], u["yaw"]]
        thruster_cmds = self.mixer.mix_from_list(control_list)

        # === 3. Публикуем UInt8MultiArray ===
        msg = UInt8MultiArray()
        msg.data = thruster_cmds
        self.pub_thruster_cmd.publish(msg)

        # === 4. Публикуем ориентацию ===
        self.pub_yaw.publish(Float64(data=self.imu.yaw))
        self.pub_pitch.publish(Float64(data=self.imu.pitch))
        self.pub_roll.publish(Float64(data=self.imu.roll))


        # Логируем статус 1 раз в секунду, а не каждый тик
        if now - self._last_status_log > self._status_log_interval:
            # self.get_logger().info(f"Thrusters command: {thruster_cmds}")
            # self.get_logger().debug(f"Control loop tick: dt={dt:.6f} s")
            self._last_status_log = now

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

    def imu_linear_accel_callback(self, msg: Vector3):
        pass
    
    def zero_yaw_callback(self, msg: Bool):
        if not msg.data:
            return

        self.yaw_zero_offset = self.imu_yaw_raw
        self.get_logger().info(
            f"Yaw zeroed at {self.yaw_zero_offset:.2f} deg")

    def pressure_sensor_callback(self, msg: Float32):
        try:
            self.depth = float(msg.data)
        except Exception as e:
            self.get_logger().warning(f"Error parsing depth msg: {e}")

    def control_data_callback(self, msg: Twist):
        self.control.impact["surge"] = float(msg.linear.x)
        self.control.impact["sway"]  = float(msg.linear.y)
        self.control.impact["heave"] = float(msg.linear.z)

        self.control.impact["roll"]  = float(msg.angular.x)
        self.control.impact["pitch"] = float(msg.angular.y)
        self.control.impact["yaw"]   = float(msg.angular.z)

    def control_mode_flags_callback(self, msg: UInt8):
        flags = int(msg.data)
        self.control.enabled["surge"] = bool(flags & (1 << 0))
        self.control.enabled["sway"]  = bool(flags & (1 << 1))
        self.control.enabled["heave"] = bool(flags & (1 << 2))
        self.control.enabled["yaw"]   = bool(flags & (1 << 3))
        self.control.enabled["pitch"] = bool(flags & (1 << 4))
        self.control.enabled["roll"]  = bool(flags & (1 << 5))

    def _on_params_changed(self, params: list[Parameter]) -> SetParametersResult:
        try:
            thruster_params: list[Parameter] = []
            controller_params: list[Parameter] = []
            other_params: list[Parameter] = []

            # Разделяем входные параметры по назначению
            for p in params:
                name = p.name

                # 1) thruster params: "<thruster>_<axis_u>"
                matched_thruster = False
                for thr in self.thrusters:
                    for a in self.axes_u:
                        if name == f"{thr}_{a}":
                            thruster_params.append(p)
                            matched_thruster = True
                            break
                    if matched_thruster:
                        break
                if matched_thruster:
                    continue

                # 2) controller params: "controllers.<axis>.<key>"
                matched_controller = False
                for axis in self.axes:
                    prefix = f'controllers.{axis}.'
                    if name.startswith(prefix):
                        controller_params.append(p)

                        # имя параметра внутри контроллера
                        key = name[len(prefix):]

                        # обновляем параметр в объекте контроллера
                        if axis in self.controllers and hasattr(self.controllers[axis], key):
                            self.controllers[axis].__dict__[key] = p.value
                            self.get_logger().info(
                                f"Controller '{axis}': {key} = {p.value}"
                            )
                        else:
                            self.get_logger().warning(
                                f"Controller '{axis}' has no param '{key}'"
                            )

                        matched_controller = True
                        break

                if matched_controller:
                    continue

                # Остальные параметры
                other_params.append(p)
                if hasattr(self, name):
                    setattr(self, name, p.value)
                    self.get_logger().info(f"Node param updated: {name} = {p.value}")

            # Сохраняем изменения в файлы конфигурации
            if thruster_params:
                try:
                    save_params(self, param_list=thruster_params, config_name="thruster_matrix")
                except Exception as e:
                    self.get_logger().error(f"Error saving thruster params: {e}")

            if controller_params:
                try:
                    save_params(self, param_list=controller_params, config_name="controllers")
                except Exception as e:
                    self.get_logger().error(f"Error saving controller params: {e}")

            if other_params:
                names = [p.name for p in other_params]
                save_params(self, param_list=other_params, config_name="stingray_core_control_node")
                self.get_logger().info(f"Other params changed (not handled specially): {names}")

            # --- Обновляем коэффициенты в mixer для изменённых thruster params ---
            if thruster_params:
                # Построим словарь частичного обновления: thruster -> row
                coeffs_update = {}
                # Инициализируем строки из текущих coeffs (если mixer есть), чтобы заполнить отсутствующие оси нулями
                current_coeffs = {}
                if hasattr(self, 'mixer') and self.mixer is not None:
                    current_coeffs = {t: list(self.mixer.coeffs.get(t, [0.0]*len(self.axes_u))) for t in self.thrusters}
                else:
                    # если mixer ещё нет — создаём шаблон с нулями
                    current_coeffs = {t: [0.0]*len(self.axes_u) for t in self.thrusters}

                for p in thruster_params:
                    # Найдём, какой thruster и какая ось
                    # формат: "<thruster>_<axis_u>" где axis_u точно равна одному из self.axes_u
                    name = p.name
                    found = False
                    for thr in self.thrusters:
                        for idx, a in enumerate(self.axes_u):
                            if name == f"{thr}_{a}":
                                # получаем значение параметра; p.value — универсально
                                try:
                                    val = float(p.value)
                                except Exception:
                                    # бывало, что числовые параметры приходят как ParameterValue объект
                                    try:
                                        val = float(p.get_parameter_value().double_value)
                                    except Exception:
                                        try:
                                            val = float(p.get_parameter_value().integer_value)
                                        except Exception:
                                            val = 0.0
                                # гарантируем наличие строки
                                row = coeffs_update.get(thr, current_coeffs.get(thr, [0.0]*len(self.axes_u))).copy()
                                row[idx] = val
                                coeffs_update[thr] = row
                                found = True
                                break
                        if found:
                            break

                # Применяем частичное обновление к mixer
                if coeffs_update:
                    try:
                        # Если mixer отсутствует — создаём новый целиком (безопасность)
                        if not hasattr(self, 'mixer') or self.mixer is None:
                            # попытка собрать полный coeffs: пробуем прочитать параметры ноды
                            full_coeffs = {}
                            for t in self.thrusters:
                                row = []
                                for a in self.axes_u:
                                    pname = f"{t}_{a}"
                                    if self.has_parameter(pname):
                                        try:
                                            pv = self.get_parameter(pname).get_parameter_value()
                                            # pv может быть integer_value или double_value
                                            if pv.type == pv.Type.INTEGER:
                                                row.append(float(pv.integer_value))
                                            else:
                                                row.append(float(pv.double_value))
                                        except Exception:
                                            row.append(0.0)
                                    else:
                                        row.append(0.0)
                                full_coeffs[t] = row
                            self.mixer = ThrusterMixer(self.thrusters, self.axes_u, full_coeffs)
                            self.get_logger().info("Mixer recreated because it was missing when params changed.")
                        else:
                            # Обновляем только изменённые строки
                            self.mixer.update_coeffs(coeffs_update)

                        # Логируем, какие thrusters обновлены
                        updated = ", ".join(f"{k}:{v}" for k, v in coeffs_update.items())
                        self.get_logger().info(f"Thruster coeffs updated: {updated}")
                    except Exception as e:
                        self.get_logger().error(f"Failed to update mixer coeffs: {e}")

            return SetParametersResult(successful=True, reason='ok')
        except Exception as e:
            self.get_logger().error(f"_on_params_changed exception: {e}")
            return SetParametersResult(successful=False, reason=str(e))
        
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

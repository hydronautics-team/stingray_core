#!/usr/bin/env python3
"""
Stingray Core Control Node
Minimal ROS2 node skeleton with 100 Hz control loop callback.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, UInt8, UInt8MultiArray

from .control.axis_controller import (
    AngularAxisController,
    AxisController,
    LinearAxisController,
    LinearPositionAxisController,
)
from .control.controllers import (
    DepthController,
    PitchController,
    RollController,
    SurgeController,
    SwayController,
    YawController,
)
from .control.thruster_mixer import ThrusterMixer
from .state.control import ControlState
from .state.imu import ImuState
from .utils.save_params import save_params

CONTROL_MODE_MANUAL = 0
CONTROL_MODE_ASSISTED = 1
CONTROL_MODE_AUTOMATIC = 2


def relative_position_target(current: float, offset: float) -> float:
    """Convert a relative linear position command to an absolute target."""
    return current + offset


def setpoint_changed(previous: float | None, current: float) -> bool:
    """Return whether a streamed setpoint represents a new command."""
    return previous is None or not math.isclose(previous, current, abs_tol=1e-9)


def body_offset_to_odom(
    surge: float, sway: float, yaw_deg: float
) -> tuple[float, float]:
    """Rotate a relative body-frame horizontal offset into the odom frame."""
    yaw = math.radians(yaw_deg)
    return (
        surge * math.cos(yaw) - sway * math.sin(yaw),
        surge * math.sin(yaw) + sway * math.cos(yaw),
    )


def odom_error_to_body(
    error_x: float, error_y: float, yaw_deg: float
) -> tuple[float, float]:
    """Rotate an odom-frame horizontal position error into the body frame."""
    yaw = math.radians(yaw_deg)
    return (
        error_x * math.cos(yaw) + error_y * math.sin(yaw),
        -error_x * math.sin(yaw) + error_y * math.cos(yaw),
    )


def horizontal_setpoint_changed(
    previous: tuple[float, float] | None, current: tuple[float, float]
) -> bool:
    """Return whether a streamed surge/sway setpoint pair changed."""
    return previous is None or any(
        not math.isclose(old, new, abs_tol=1e-9)
        for old, new in zip(previous, current)
    )


class StingrayCoreControlNode(Node):
    def __init__(self):
        super().__init__("stingray_core_control_node")
        self._init_config()
        self._init_control_core()
        self._init_ros_interfaces()
        self._init_runtime()

        self.get_logger().info(
            f"Started at {self.rate_hz} Hz | axes={self.axes} | thrusters={len(self.thrusters)}"
        )
        self.get_logger().info(
            f"State input: imu={self.topic_imu} | odometry={self.topic_odometry}"
        )

    def control_loop(self):
        now = time.time()
        dt = now - self.last_time if self.last_time is not None else 0.0
        self.last_time = now
        self.last_dt = max(min(dt, 0.05), 1e-3)

        monotonic_now = time.monotonic()
        timed_out_axes = []
        for axis in self.axes:
            last_axis_command = self.last_control_input_time[axis]
            if (
                last_axis_command > 0.0
                and monotonic_now - last_axis_command > self.command_timeout_sec
                and self.control_input[axis] != 0.0
            ):
                self.control_input[axis] = 0.0
                timed_out_axes.append(axis)
        if timed_out_axes:
            self.get_logger().warning(
                "Control command timeout; stopping axes: " + ", ".join(timed_out_axes)
            )

        # === 1. Определяем управляющие воздействия ===
        u: dict[str, float] = {}
        horizontal_error_body = odom_error_to_body(
            self.control_setpoint["surge"] - self.dvl_position_x,
            self.control_setpoint["sway"] - self.dvl_position_y,
            self.imu.yaw,
        )

        assisted_mode = self.control_mode == CONTROL_MODE_ASSISTED
        sum_mode_active = assisted_mode and self.sum_control_impacts
        if sum_mode_active:
            horizontal_active = any(
                abs(self.control_input[axis]) > self.manual_impact_deadband
                for axis in ("surge", "sway")
            )
            horizontal_was_active = any(
                self.manual_impact_active[axis] for axis in ("surge", "sway")
            )
            if horizontal_active or horizontal_was_active:
                self._capture_horizontal_hold_setpoint()
            self.manual_impact_active["surge"] = horizontal_active
            self.manual_impact_active["sway"] = horizontal_active

        for axis in self.axes:
            if self.control_mode == CONTROL_MODE_MANUAL:
                u[axis] = self.control_input[axis]
                self.control.impact[axis] = u[axis]
                continue

            if axis in self.axis_ctrl and self.control.enabled[axis]:
                manual_impact = self.control_input[axis] if assisted_mode else 0.0
                if sum_mode_active and axis not in ("surge", "sway"):
                    manual_active = (
                        abs(manual_impact) > self.manual_impact_deadband
                    )
                    if manual_active or self.manual_impact_active[axis]:
                        self._capture_hold_setpoint(axis)
                    self.manual_impact_active[axis] = manual_active

                if axis in ("surge", "sway"):
                    if (
                        not self.position_target_initialized[axis]
                        or not self._is_dvl_fresh()
                        or not self.dvl_velocity_valid
                        or (
                            not self.flag_setup_feedback_speed
                            and not self._is_dvl_position_fresh()
                        )
                    ):
                        # Both DVL position and velocity are mandatory for the
                        # horizontal position loops.
                        # In assisted mode the operator must retain direct
                        # control even if localization is temporarily invalid.
                        u[axis] = manual_impact if assisted_mode else 0.0
                        self.control.impact[axis] = u[axis]
                        continue

                if self.flag_setup_feedback_speed:
                    # Tuning mode is the only mode where /control/data carries
                    # velocity setpoints instead of position/angle setpoints.
                    self.control_setpoint[axis] = self.control_input[axis]

                target = self.control_setpoint[axis]
                if axis == "surge" and not self.flag_setup_feedback_speed:
                    target = horizontal_error_body[0]
                elif axis == "sway" and not self.flag_setup_feedback_speed:
                    target = horizontal_error_body[1]

                u[axis] = self.axis_ctrl[axis].compute(target=target, dt=self.last_dt)
                if assisted_mode:
                    # With summing disabled the hold point remains fixed, so
                    # manual impact only displaces the vehicle temporarily.
                    # With summing enabled the hold point follows the vehicle
                    # and is captured once more when the operator releases it.
                    u[axis] += manual_impact
            else:
                u[axis] = self.control_input[axis]
                # # Для разомкнутого контура воздействие одноразовое: 1 цикл,
                # # затем сбрасывается до следующей новой команды.
                # if self.open_loop_pending[axis]:
                #     u[axis] = self.control_input[axis]
                #     self.open_loop_pending[axis] = False
                # else:
                #     u[axis] = 0.0

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
        self.declare_parameter("rate_hz", 100.0)
        self.rate_hz = float(self.get_parameter("rate_hz").value)

        self.declare_parameter("flag_setup_feedback_speed", False)
        self.flag_setup_feedback_speed = bool(
            self.get_parameter("flag_setup_feedback_speed").value
        )

        self.declare_parameter("debug_publish", False)

        defaults = {
            "topic_imu": "/core/sensors/imu",
            "topic_odometry": "/core/state/odometry",
            "topic_loop_flags": "/control/loop_flags",
            "topic_control_data": "/control/data",
            "topic_control_mode": "/control/mode",
        }

        for name, default in defaults.items():
            self.declare_parameter(name, default)
            setattr(self, name, self.get_parameter(name).value)

        self.declare_parameter(
            "axes", ["surge", "sway", "heave", "roll", "pitch", "yaw"]
        )
        self.axes = list(self.get_parameter("axes").value)

        self.declare_parameter("dvl_timeout_sec", 0.5)
        self.dvl_timeout_sec = float(self.get_parameter("dvl_timeout_sec").value)

        self.declare_parameter("dvl_position_timeout_sec", 0.5)
        self.dvl_position_timeout_sec = float(
            self.get_parameter("dvl_position_timeout_sec").value
        )

        self.declare_parameter("command_timeout_sec", 0.5)
        self.command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )

        self.declare_parameter("manual_impact_deadband", 1e-3)
        self.manual_impact_deadband = float(
            self.get_parameter("manual_impact_deadband").value
        )

        self.declare_parameter("thrusters", Parameter.Type.STRING_ARRAY)
        self.thrusters = list(self.get_parameter("thrusters").value)

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
            "yaw": YawController,
            "pitch": PitchController,
            "roll": RollController,
            "heave": DepthController,
            "surge": SurgeController,
            "sway": SwayController,
        }

        self.controllers = {}
        self.param_keys = [
            "K_p",
            "K_1",
            "K_2",
            "K_i",
            "I_min",
            "I_max",
            "out_sat",
            "ap_K",
            "ap_T",
        ]
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

        for axis, ctrl in self.controllers.items():
            ctrl.set_debug_hook(
                lambda data, axis=axis: self.debug_cb(axis, data)
            )

        self.imu = ImuState()
        self.control = ControlState.from_axes(self.axes)

        # Последняя полученная команда из /control/data
        self.control_input: dict[str, float] = {axis: 0.0 for axis in self.axes}
        self.control_mode = CONTROL_MODE_MANUAL
        self.sum_control_impacts = False
        self.manual_impact_active: dict[str, bool] = {
            axis: False for axis in self.axes
        }
        # Удерживаемые цели для замкнутых контуров
        self.control_setpoint: dict[str, float] = {axis: 0.0 for axis in self.axes}
        # Флаг одноразовой подачи для разомкнутых контуров
        self.open_loop_pending: dict[str, bool] = {axis: False for axis in self.axes}

        self.depth = 0.0
        self.depth_rate = 0.0
        self.previous_depth = None
        self.previous_depth_stamp = None
        self.distance_to_bottom = 0.0

        self.dvl_velocity_x = 0.0
        self.dvl_velocity_y = 0.0
        self.dvl_velocity_z = 0.0
        self.dvl_velocity_valid = False
        self.dvl_last_time = 0.0
        self.dvl_position_x = 0.0
        self.dvl_position_y = 0.0
        self.dvl_position_valid = False
        self.dvl_position_last_time = 0.0
        self.position_target_initialized = {"surge": False, "sway": False}
        self.last_control_input_time: dict[str, float] = {
            axis: 0.0 for axis in self.axes
        }
        self.last_closed_loop_command: dict[str, float | None] = {
            axis: None for axis in self.axes
        }
        self.last_horizontal_command: tuple[float, float] | None = None

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
            "surge": LinearPositionAxisController(
                controller=self.controllers["surge"],
                pos_fn=lambda: 0.0,
                vel_fn=self._get_surge_velocity_measurement,
                feedback_flag_fn=lambda: self.flag_setup_feedback_speed,
            ),
            "sway": LinearPositionAxisController(
                controller=self.controllers["sway"],
                pos_fn=lambda: 0.0,
                vel_fn=self._get_sway_velocity_measurement,
                feedback_flag_fn=lambda: self.flag_setup_feedback_speed,
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

    def _is_dvl_position_fresh(self) -> bool:
        if not self.dvl_position_valid or self.dvl_position_last_time <= 0.0:
            return False
        return (
            time.time() - self.dvl_position_last_time
        ) <= self.dvl_position_timeout_sec

    def _get_surge_velocity_measurement(self) -> float:
        return self.dvl_velocity_x

    def _get_sway_velocity_measurement(self) -> float:
        return self.dvl_velocity_y

    def _get_heave_velocity_measurement(self) -> float:
        return self.depth_rate

    def _capture_horizontal_hold_setpoint(self) -> bool:
        """Capture the current odom X/Y as one horizontal hold target."""
        if not self._is_dvl_position_fresh():
            return False
        self.control_setpoint["surge"] = self.dvl_position_x
        self.control_setpoint["sway"] = self.dvl_position_y
        for axis in ("surge", "sway"):
            if self.control.enabled.get(axis, False):
                self.position_target_initialized[axis] = True
                self.controllers[axis].reset()
        return True

    def _capture_hold_setpoint(self, axis: str) -> bool:
        """Capture the current state as an enabled loop's hold target."""
        if axis in ("surge", "sway"):
            return self._capture_horizontal_hold_setpoint()
        if axis == "heave":
            self.control_setpoint[axis] = self.depth
        elif axis == "yaw":
            self.control_setpoint[axis] = self.imu.yaw
        elif axis == "pitch":
            self.control_setpoint[axis] = self.imu.pitch
        elif axis == "roll":
            self.control_setpoint[axis] = self.imu.roll
        else:
            return False
        self.controllers[axis].reset()
        return True

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

        self.sub_imu = self.create_subscription(
            Imu, self.topic_imu, self.imu_callback, qos_sensor
        )

        self.sub_odometry = self.create_subscription(
            Odometry, self.topic_odometry, self.odometry_callback, qos_sensor
        )

        self.sub_control_mode_flags = self.create_subscription(
            UInt8, self.topic_loop_flags, self.control_mode_flags_callback, qos_command
        )

        self.sub_control_data = self.create_subscription(
            Twist, self.topic_control_data, self.control_data_callback, qos_command
        )

        self.sub_control_mode = self.create_subscription(
            UInt8, self.topic_control_mode, self.control_mode_callback, qos_command
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
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        qos_debug = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.pub_thruster_cmd = self.create_publisher(
            UInt8MultiArray, "/thruster/cmd", qos_actuation
        )

        self.pub_yaw = self.create_publisher(
            Float64, "~/orientation/yaw", qos_telemetry
        )
        self.pub_pitch = self.create_publisher(
            Float64, "~/orientation/pitch", qos_telemetry
        )
        self.pub_roll = self.create_publisher(
            Float64, "~/orientation/roll", qos_telemetry
        )
        self.pub_depth = self.create_publisher(
            Float64, "~/orientation/depth", qos_telemetry
        )

        self.pub_err_position = self.create_publisher(
            Float64, "~/debug/err_position", qos_debug
        )
        self.pub_output_pi = self.create_publisher(
            Float64, "~/debug/output_pi", qos_debug
        )
        self.pub_feedback_speed = self.create_publisher(
            Float64, "~/debug/feedback_speed", qos_debug
        )
        self.pub_measurement_rate = self.create_publisher(
            Float64, "~/debug/measurement_rate", qos_debug
        )
        self.pub_out = self.create_publisher(Float64, "~/debug/out", qos_debug)

        debug_fields = (
            "err_position",
            "output_pi",
            "feedback_speed",
            "measurement_rate",
            "out",
        )
        self.axis_debug_publishers = {
            axis: {
                field: self.create_publisher(
                    Float64, f"~/debug/{axis}/{field}", qos_debug
                )
                for field in debug_fields
            }
            for axis in self.axes
        }

    # --- Колбэки ---
    def imu_callback(self, msg: Imu):
        """Consume the sensor-independent IMU stream produced by localization."""
        try:
            qx = float(msg.orientation.x)
            qy = float(msg.orientation.y)
            qz = float(msg.orientation.z)
            qw = float(msg.orientation.w)

            norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
            if norm < 1e-9:
                self.get_logger().warning("Localization supplied a zero quaternion")
                return

            qx /= norm
            qy /= norm
            qz /= norm
            qw /= norm

            sin_roll_cos_pitch = 2.0 * (qw * qx + qy * qz)
            cos_roll_cos_pitch = 1.0 - 2.0 * (qx * qx + qy * qy)
            roll_deg = math.degrees(math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch))

            sin_pitch = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
            pitch_deg = math.degrees(math.asin(sin_pitch))

            sin_yaw_cos_pitch = 2.0 * (qw * qz + qx * qy)
            cos_yaw_cos_pitch = 1.0 - 2.0 * (qy * qy + qz * qz)
            yaw_deg = math.degrees(math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch))

            self.imu_yaw_raw = yaw_deg
            self.imu.yaw = self.normalize_angle_deg(yaw_deg)
            self.imu.pitch = pitch_deg
            self.imu.roll = roll_deg

            self.imu.rate_x = float(msg.angular_velocity.x)
            self.imu.rate_y = float(msg.angular_velocity.y)
            self.imu.rate_z = float(msg.angular_velocity.z)

            self.imu.accel_x = float(msg.linear_acceleration.x)
            self.imu.accel_y = float(msg.linear_acceleration.y)
            self.imu.accel_z = float(msg.linear_acceleration.z)
        except Exception as e:
            self.get_logger().warning(f"Error parsing standardized Imu message: {e}")

    def odometry_callback(self, msg: Odometry):
        """Consume fused position, depth and velocity from localization."""
        try:
            self.dvl_velocity_x = float(msg.twist.twist.linear.x)
            self.dvl_velocity_y = float(msg.twist.twist.linear.y)
            self.dvl_velocity_z = float(msg.twist.twist.linear.z)
            self.dvl_velocity_valid = all(
                math.isfinite(value)
                for value in (
                    self.dvl_velocity_x,
                    self.dvl_velocity_y,
                    self.dvl_velocity_z,
                )
            ) and max(msg.twist.covariance[0], msg.twist.covariance[7]) < 100.0
            self.dvl_last_time = time.time()

            self.dvl_position_x = float(msg.pose.pose.position.x)
            self.dvl_position_y = float(msg.pose.pose.position.y)
            self.dvl_position_valid = all(
                math.isfinite(value)
                for value in (self.dvl_position_x, self.dvl_position_y)
            ) and max(msg.pose.covariance[0], msg.pose.covariance[7]) < 100.0
            self.dvl_position_last_time = time.time()
            # REP-103 odometry Z is positive upward; control depth and its
            # rate are positive down. Calculate the rate once per new odometry
            # sample, using the message timestamp rather than the 100 Hz loop.
            new_depth = -float(msg.pose.pose.position.z)
            stamp = (
                float(msg.header.stamp.sec)
                + float(msg.header.stamp.nanosec) * 1e-9
            )
            if stamp <= 0.0:
                stamp = time.monotonic()

            if (
                self.previous_depth is not None
                and self.previous_depth_stamp is not None
            ):
                depth_dt = stamp - self.previous_depth_stamp
                if 1e-4 <= depth_dt <= 1.0:
                    self.depth_rate = (
                        new_depth - self.previous_depth
                    ) / depth_dt
                else:
                    self.depth_rate = 0.0

            self.depth = new_depth
            self.previous_depth = new_depth
            self.previous_depth_stamp = stamp

            horizontal_needs_initialization = any(
                self.control.enabled.get(axis, False)
                and not self.position_target_initialized[axis]
                for axis in ("surge", "sway")
            )
            if not self.flag_setup_feedback_speed and horizontal_needs_initialization:
                # X/Y form one odom-frame target even if only one horizontal
                # controller is enabled, so initialize them atomically.
                self.control_setpoint["surge"] = self.dvl_position_x
                self.control_setpoint["sway"] = self.dvl_position_y
                for axis in ("surge", "sway"):
                    if self.control.enabled.get(axis, False):
                        self.position_target_initialized[axis] = True
        except Exception as e:
            self.get_logger().warning(f"Error parsing standardized odometry: {e}")

    def control_data_callback(self, msg: Twist):
        if self.control_mode in (CONTROL_MODE_MANUAL, CONTROL_MODE_ASSISTED):
            self._process_manual_impact(msg)
            return
        self._process_position_setpoint(msg)

    def _process_manual_impact(self, msg: Twist):
        """Store UMAS-GUI commands as direct axis impacts."""
        command_time = time.monotonic()
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
            self.last_control_input_time[axis] = command_time

    def _process_position_setpoint(self, msg: Twist):
        """Resolve automatic relative-position and absolute-angle setpoints."""
        command_time = time.monotonic()
        incoming = {
            "surge": float(msg.linear.x),
            "sway": float(msg.linear.y),
            "heave": float(msg.linear.z),
            "roll": float(msg.angular.x),
            "pitch": float(msg.angular.y),
            "yaw": float(msg.angular.z),
        }

        horizontal_command = (incoming["surge"], incoming["sway"])
        horizontal_closed = any(
            self.control.enabled.get(axis, False) for axis in ("surge", "sway")
        )
        if (
            horizontal_closed
            and not self.flag_setup_feedback_speed
            and horizontal_setpoint_changed(
                self.last_horizontal_command, horizontal_command
            )
        ):
            if self._is_dvl_position_fresh():
                offset_x, offset_y = body_offset_to_odom(
                    horizontal_command[0], horizontal_command[1], self.imu.yaw
                )
                targets = {
                    "surge": self.dvl_position_x + offset_x,
                    "sway": self.dvl_position_y + offset_y,
                }
                for horizontal_axis in ("surge", "sway"):
                    # These two fields store the target X/Y coordinates in
                    # odom; both coordinates belong to one atomic target.
                    self.control_setpoint[horizontal_axis] = targets[horizontal_axis]
                    if self.control.enabled.get(horizontal_axis, False):
                        self.position_target_initialized[horizontal_axis] = True
                        self.controllers[horizontal_axis].reset()
                self.last_horizontal_command = horizontal_command
            else:
                self.get_logger().warning(
                    "Ignored horizontal setpoint: odometry position is stale"
                )

        for axis, value in incoming.items():
            if not self.control.enabled.get(axis, False):
                self.control_input[axis] = value
                self.last_control_input_time[axis] = command_time
                continue

            if self.flag_setup_feedback_speed:
                # In tuning mode all six fields are direct velocity setpoints.
                self.control_input[axis] = value
                self.last_control_input_time[axis] = command_time
                continue

            if axis in ("surge", "sway"):
                # The horizontal pair was resolved atomically above so that
                # body-frame march/lag can be rotated into the odom frame.
                self.control_input[axis] = 0.0
                continue

            # Closed position/attitude loops hold the resolved target and do
            # not integrate /control/data in the periodic control loop.
            self.control_input[axis] = 0.0
            if not setpoint_changed(self.last_closed_loop_command[axis], value):
                continue

            if axis == "heave":
                # Depth is positive down; linear.z is a relative displacement.
                self.control_setpoint[axis] = relative_position_target(
                    self.depth, value
                )
            else:
                # angular.x/y/z are absolute attitude setpoints in degrees.
                self.control_setpoint[axis] = self.normalize_angle_deg(value)

            self.last_closed_loop_command[axis] = value
            self.controllers[axis].reset()

    def control_mode_callback(self, msg: UInt8):
        requested_mode = int(msg.data)
        if requested_mode not in (
            CONTROL_MODE_MANUAL,
            CONTROL_MODE_ASSISTED,
            CONTROL_MODE_AUTOMATIC,
        ):
            self.get_logger().warning(
                f"Ignored invalid control mode: {requested_mode}"
            )
            return
        if requested_mode == self.control_mode:
            return

        old_mode = self.control_mode
        self.control_mode = requested_mode
        for axis in self.axes:
            self.control_input[axis] = 0.0
            self.manual_impact_active[axis] = False
            self.last_closed_loop_command[axis] = None
        self.last_horizontal_command = None

        if requested_mode != CONTROL_MODE_MANUAL:
            self._capture_horizontal_hold_setpoint()
            for axis in ("heave", "yaw"):
                if self.control.enabled.get(axis, False):
                    self._capture_hold_setpoint(axis)
            for axis in ("pitch", "roll"):
                if self.control.enabled.get(axis, False):
                    self.control_setpoint[axis] = 0.0
                    self.controllers[axis].reset()

        self.get_logger().info(
            f"Control mode changed: {old_mode} -> {self.control_mode}"
        )

    def control_mode_flags_callback(self, msg: UInt8):
        flags = int(msg.data)
        new_sum_control_impacts = bool(flags & (1 << 6))
        sum_mode_changed = new_sum_control_impacts != self.sum_control_impacts
        self.sum_control_impacts = new_sum_control_impacts
        new_enabled = {
            "surge": bool(flags & (1 << 0)),
            "sway": bool(flags & (1 << 1)),
            "heave": bool(flags & (1 << 2)),
            "yaw": bool(flags & (1 << 3)),
            "pitch": bool(flags & (1 << 4)),
            "roll": bool(flags & (1 << 5)),
        }

        changed = []
        newly_enabled = set()
        for axis, new_value in new_enabled.items():
            old_value = bool(self.control.enabled.get(axis, False))
            if old_value != new_value:
                changed.append(f"{axis}: {old_value} -> {new_value}")

                # if new_value:
                #     # При включении контура берём последнюю принятую команду как цель.
                #     self.control_setpoint[axis] = self.control_input[axis]
                if new_value:
                    newly_enabled.add(axis)
                    self.last_closed_loop_command[axis] = None
                    if axis in ("surge", "sway"):
                        self.last_horizontal_command = None
                    if axis == "heave":
                        self.control_input[axis] = 0.0
                        self.control_setpoint[axis] = self.depth
                        self.controllers["heave"].reset()
                    elif axis == "yaw":
                        self.control_setpoint[axis] = self.imu.yaw
                    elif axis == "pitch":
                        self.control_setpoint[axis] = 0.0
                        self.controllers["pitch"].reset()
                    elif axis == "roll":
                        self.control_setpoint[axis] = 0.0
                        self.controllers["roll"].reset()
                    elif axis == "surge":
                        self.control_input[axis] = 0.0
                        if self.flag_setup_feedback_speed:
                            self.control_setpoint[axis] = 0.0
                            self.position_target_initialized[axis] = True
                        elif self._is_dvl_position_fresh():
                            self.control_setpoint["surge"] = self.dvl_position_x
                            self.control_setpoint["sway"] = self.dvl_position_y
                            self.position_target_initialized[axis] = True
                        else:
                            self.position_target_initialized[axis] = False
                        self.controllers[axis].reset()
                    elif axis == "sway":
                        self.control_input[axis] = 0.0
                        if self.flag_setup_feedback_speed:
                            self.control_setpoint[axis] = 0.0
                            self.position_target_initialized[axis] = True
                        elif self._is_dvl_position_fresh():
                            self.control_setpoint["surge"] = self.dvl_position_x
                            self.control_setpoint["sway"] = self.dvl_position_y
                            self.position_target_initialized[axis] = True
                        else:
                            self.position_target_initialized[axis] = False
                        self.controllers[axis].reset()
                else:
                    # При выключении контура не повторяем старую команду.
                    self.control_input[axis] = 0.0
                    self.open_loop_pending[axis] = False
                    if axis in self.position_target_initialized:
                        self.position_target_initialized[axis] = False
                    self.last_closed_loop_command[axis] = None
                    if axis in ("surge", "sway"):
                        self.last_horizontal_command = None

            self.control.enabled[axis] = new_value

        if sum_mode_changed:
            # Do not carry a manual impact across a semantics switch. Capture
            # the current point to avoid a jump when summing is toggled.
            for axis in self.axes:
                self.control_input[axis] = 0.0
                self.manual_impact_active[axis] = False
            self._capture_horizontal_hold_setpoint()
            for axis in ("heave", "yaw", "pitch", "roll"):
                if self.control.enabled.get(axis, False):
                    # A pitch/roll loop closed by this same flags message must
                    # keep its mandatory zero-degree target.
                    if axis in ("pitch", "roll") and axis in newly_enabled:
                        continue
                    self._capture_hold_setpoint(axis)
            changed.append(
                "sum_control_impacts: "
                f"{not self.sum_control_impacts} -> {self.sum_control_impacts}"
            )

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
                    self.get_logger().info(f"Controller '{axis}': {key} = {p.value}")
                else:
                    self.get_logger().warning(f"Ignored controller param: {p.name}")

            # --- 3. Применяем node params ---
            for p in node_params:
                if hasattr(self, p.name):
                    setattr(self, p.name, p.value)
                    self.get_logger().info(f"Node param updated: {p.name} = {p.value}")

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
            self.get_logger().info(f"Updated thruster coeffs: {coeffs_update}")

    def _is_thruster_param(self, name: str) -> bool:
        return any(
            name.startswith(f"{thr}_") and name[len(thr) + 1 :] in self.axes
            for thr in self.thrusters
        )

    def _is_controller_param(self, name: str) -> bool:
        return name.startswith("controllers.")

    def debug_cb(self, axis: str, data: dict):
        if not self.get_parameter("debug_publish").value:
            return

        publishers = self.axis_debug_publishers[axis]
        for field, publisher in publishers.items():
            publisher.publish(Float64(data=float(data[field])))

        # Keep the old topics as heave-only aliases. They are no longer
        # overwritten by whichever angular controller runs last.
        if axis == "heave":
            self.pub_err_position.publish(Float64(data=data["err_position"]))
            self.pub_output_pi.publish(Float64(data=data["output_pi"]))
            self.pub_feedback_speed.publish(Float64(data=data["feedback_speed"]))
            self.pub_measurement_rate.publish(
                Float64(data=data["measurement_rate"])
            )
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


if __name__ == "__main__":
    main()

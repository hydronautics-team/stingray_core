# controllers.py
from abc import ABC, abstractmethod
import math
from typing import Optional

# -----------------------
# small helpers
# -----------------------
def saturation(value: float, upper: Optional[float], lower: Optional[float]) -> float:
    """Насыщение с поддержкой None (без ограничения)."""
    if upper is not None and value > upper:
        value = upper
    if lower is not None and value < lower:
        value = lower
    return value

def normalize_angle_deg(angle_deg: float) -> float:
    """Нормализует угол в градусах в диапазон (-180, 180]."""
    a = (angle_deg + 180.0) % 360.0
    if a <= 0.0:
        a += 360.0
    return a - 180.0

# -----------------------
# BaseController (abstract)
# -----------------------
class BaseController(ABC):
    """
    Абстрактный базовый контроллер.
    Параметры (основные):
      - K_p  : коэффициент P (вспомогательный)
      - K_1  : стадийный множитель
      - K_2  : множитель ап-части / обратной связи по скорости
      - K_i  : множитель интегратора (интегрируем stage * K_i)
      - I_max, I_min : пределы интегратора (anti-windup)
      - out_sat : предел выхода (симметричный, если positive)
      - ap_K, ap_T : параметры апериодического фильтра (K, T). Если ap_T == 0 -> апериодика вырождается в K * input
    """
    def __init__(self,
                 K_p: float = 1.0,
                 K_1: float = 1.0,
                 K_2: float = 1.0,
                 K_i: float = 1.0,
                 I_max: Optional[float] = None,
                 I_min: Optional[float] = None,
                 out_sat: Optional[float] = None,
                 ap_K: float = 1.0,
                 ap_T: float = 0.0):
        # gains
        self.K_p = float(K_p)
        self.K_1 = float(K_1)
        self.K_2 = float(K_2)
        self.K_i = float(K_i)
        # integrator limits
        self.I_max = I_max
        self.I_min = I_min
        # output saturation
        self.out_sat = out_sat
        # aperiodic filter params
        self.ap_K = float(ap_K)
        self.ap_T = float(ap_T)

        # internal states
        self.integrator = 0.0     
        self.prev_stage = 0.0     
        self.ap_out = 0.0         

    def reset(self):
        """Сброс внутренних состояний (интегратор, апериодика)."""
        self.integrator = 0.0
        self.prev_stage = 0.0
        self.ap_out = 0.0

    def trapezoidal_integrate(self, stage: float, dt: float) -> float:
        """Трапециевидная интеграция + anti-windup лимиты."""
        self.integrator += 0.5 * (stage + self.prev_stage) * dt
        # anti-windup
        if (self.I_max is not None) or (self.I_min is not None):
            hi = self.I_max if self.I_max is not None else float('inf')
            lo = self.I_min if self.I_min is not None else -float('inf')
            self.integrator = saturation(self.integrator, hi, lo)
        self.prev_stage = stage
        return self.integrator

    def aperiodic_step(self, input_val: float, dt: float) -> float:
        """Апериодический фильтр первого порядка.
           Если ap_T == 0 -> работает как ap_K * input (выпрямление)."""
        if self.ap_T != 0.0:
            # dx/dt = (K * input - x) / T  -> x += dt * (1/T) * (K*input - x)
            self.ap_out += dt * (1.0 / self.ap_T) * (input_val * self.ap_K - self.ap_out)
        else:
            self.ap_out = self.ap_K * input_val
        return self.ap_out

    @abstractmethod
    def update(self, setpoint: float, measurement: float, measurement_rate: float, dt: float) -> float:
        """
        Главный метод: возвращает управляющий сигнал (без прямого маппинга на thrusters).
        Подклассы обязаны реализовать.
        - setpoint: желаемая величина (угол в deg для угловых контроллеров)
        - measurement: текущее измерение (deg для углов)
        - measurement_rate: скорость (deg/s для углов, единицы для линейных)
        - dt: шаг времени в секундах
        """
        raise NotImplementedError

# -----------------------
# Angular controllers
# -----------------------
class YawController(BaseController):
    def update(self, setpoint: float, measurement: float, measurement_rate: float, dt: float, flag_setup_feedback_speed: bool) -> float:
        setspeed = setpoint * flag_setup_feedback_speed
        if flag_setup_feedback_speed:
            output_pi = 0
        else:
            # 1) ошибка (с учётом wrap-around)
            err_position = normalize_angle_deg(setpoint - measurement)

            # 2) stage (предобработка)  = err * K_1
            stage = err_position * self.K_1

            # 3) P part (stage scaled by K_p)
            res_p = stage * self.K_p

            # 4) I part (integrate stage * K_i)
            res_i = self.trapezoidal_integrate(stage * self.K_xi, dt)

            # 5) PI combined
            output_pi = res_p + res_i

        # 6) aperiodic filter from rate (wz) and scale
        ap = self.aperiodic_step(measurement_rate, dt)
        feedback_speed = ap * self.K_2

        # 7) final output (PI minus speed feedback) 
        error_speed = output_pi + setspeed - feedback_speed 

        # 8) saturate final output symmetrically by out_sat (if provided)
        if self.out_sat is not None:
            out = saturation(error_speed, self.out_sat, -self.out_sat)
        return out

class PitchController(BaseController):
    def update(self, setpoint: float, measurement: float, measurement_rate: float, dt: float) -> float:
        # same structure as yaw but error normalized for angles
        err = normalize_angle_deg(setpoint - measurement)
        stage = err * self.K_1
        res_p = stage * self.K_p
        res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
        output_pi = res_p + res_i
        ap = self.aperiodic_step(measurement_rate, dt)
        feedback_speed = ap * self.K_2
        out = output_pi - feedback_speed
        if self.out_sat is not None:
            out = saturation(out, self.out_sat, -self.out_sat)
        return out

class RollController(BaseController):
    def update(self, setpoint: float, measurement: float, measurement_rate: float, dt: float) -> float:
        # same as pitch
        err = normalize_angle_deg(setpoint - measurement)
        stage = err * self.K_1
        res_p = stage * self.K_p
        res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
        output_pi = res_p + res_i
        ap = self.aperiodic_step(measurement_rate, dt)
        feedback_speed = ap * self.K_2
        out = output_pi - feedback_speed
        if self.out_sat is not None:
            out = saturation(out, self.out_sat, -self.out_sat)
        return out

# -----------------------
# Linear controllers
# -----------------------
class DepthController(BaseController):
    def update(self, setpoint: float, measurement: float, measurement_rate: float, dt: float) -> float:
        # setpoint, measurement in meters (or same unit)
        err = setpoint - measurement
        # stage from err
        stage = err * self.K_1
        # P
        res_p = stage * self.K_p
        # I
        res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
        output_pi = res_p + res_i
        # aperiodic from vertical speed (measurement_rate)
        ap = self.aperiodic_step(measurement_rate, dt)
        feedback_speed = ap * self.K_2
        out = output_pi - feedback_speed
        if self.out_sat is not None:
            out = saturation(out, self.out_sat, -self.out_sat)
        return out

class SurgeController(BaseController):
    """Controller for surge-like channel. Simpler: optionally only P or PI."""
    def update(self, setpoint: float, measurement: float, measurement_rate: float, dt: float) -> float:
        err = setpoint - measurement
        # simple PD/PI depending on gains: use K_1 as stage multiplier
        stage = err * self.K_1
        res_p = stage * self.K_p
        res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
        output_pi = res_p + res_i
        # optional aperiodic (often not used for surge)
        ap = self.aperiodic_step(measurement_rate, dt)
        feedback_speed = ap * self.K_2
        out = output_pi - feedback_speed
        if self.out_sat is not None:
            out = saturation(out, self.out_sat, -self.out_sat)
        return out

class SwayController(BaseController):
    """Controller for sway-like channel. Mirror of Ux."""
    def update(self, setpoint: float, measurement: float, measurement_rate: float, dt: float) -> float:
        err = setpoint - measurement
        stage = err * self.K_1
        res_p = stage * self.K_p
        res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
        output_pi = res_p + res_i
        ap = self.aperiodic_step(measurement_rate, dt)
        feedback_speed = ap * self.K_2
        out = output_pi - feedback_speed
        if self.out_sat is not None:
            out = saturation(out, self.out_sat, -self.out_sat)
        return out

# -----------------------
# End of file
# -----------------------

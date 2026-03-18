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

        self.debug_hook = None    

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
    
    def set_debug_hook(self, hook):
        """hook(dict) — вызывается, если debug включён"""
        self.debug_hook = hook

    def apply_params(self, **kwargs):
        """
        Обновляет параметры контроллера "на лету".
        Не трогает внутренние состояния, если явно не указано reset=True.
        """
        reset_required = False

        for k, v in kwargs.items():
            if not hasattr(self, k):
                continue

            # если меняем динамические параметры — нужен reset
            if k in ("K_i", "ap_T", "ap_K"):
                reset_required = True

            setattr(self, k, float(v) if isinstance(v, (int, float)) else v)

        if reset_required:
            self.reset()



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
    def update(
        self,
        setpoint: float,
        measurement: float,
        measurement_rate: float,
        dt: float,
        flag_setup_feedback_speed: bool,
        param_update: dict | None = None
    ) -> float:

        if param_update:
            self.apply_params(**param_update)

        setspeed = setpoint * flag_setup_feedback_speed
        if flag_setup_feedback_speed:
            output_pi = 0.0
            err_position = 0.0
        else:
            # 1) ошибка (с учётом wrap-around)
            err_position = normalize_angle_deg(setpoint - measurement)

            # 2) stage (предобработка)  = err * K_1
            stage = err_position * self.K_1

            # 3) P part (stage scaled by K_p)
            res_p = stage * self.K_p

            # 4) I part (integrate stage * K_i)
            res_i = self.trapezoidal_integrate(stage * self.K_i, dt)

            # 5) PI combined
            output_pi = res_p + res_i

        # 6) aperiodic filter from rate (wz) and scale
        ap = self.aperiodic_step(57.3 * measurement_rate, dt)
        feedback_speed = ap * self.K_2
        # feedback_speed = 57.3 *measurement_rate * self.K_2

        # 7) final output (PI minus speed feedback) 
        error_speed = output_pi + setspeed - feedback_speed 

        # 8) saturate final output symmetrically by out_sat (if provided)
        if self.out_sat is not None:
            out = saturation(error_speed, self.out_sat, -self.out_sat)
        else:
            out = error_speed

        if self.debug_hook is not None:
            self.debug_hook({
                "err_position": err_position,
                "output_pi": output_pi,
                "feedback_speed": feedback_speed,
                "measurement_rate": 57.3 *measurement_rate,
                "out": out,
            })

        return out

class PitchController(BaseController):
    def update(
        self,
        setpoint: float,
        measurement: float,
        measurement_rate: float,
        dt: float,
        flag_setup_feedback_speed: bool,
        param_update: dict | None = None
    ) -> float:

        if param_update:
            self.apply_params(**param_update)

        dt = max(min(dt, 0.05), 1e-3)

        # --- позиционный контур ---
        if flag_setup_feedback_speed:
            err_position = 0.0
            output_pi = 0.0
            setspeed = setpoint
        else:
            err_position = normalize_angle_deg(setpoint - measurement)
            stage = err_position * self.K_1
            res_p = stage * self.K_p
            res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
            output_pi = res_p + res_i
            setspeed = 0.0

        # --- ОС по скорости ---
        ap = self.aperiodic_step(57.3 *measurement_rate, dt)
        feedback_speed = ap * self.K_2

        error_speed = output_pi + setspeed - feedback_speed

        if self.out_sat is not None:
            out = saturation(error_speed, self.out_sat, -self.out_sat)
        else:
            out = error_speed

        # --- DEBUG ---
        if self.debug_hook is not None:
            self.debug_hook({
                "err_position": err_position,
                "output_pi": output_pi,
                "measurement_rate": 57.3 *measurement_rate,
                "feedback_speed": feedback_speed,
                "error_speed": error_speed,
                "out": out,
            })

        return out

class RollController(BaseController):
    def __init__(
        self,
        *args,
        grav_bias: float = 0.0,
        grav_gain: float = 0.0,
        grav_offset_deg: float = 0.0,
        **kwargs
    ):
        super().__init__(*args, **kwargs)
        self.grav_bias = grav_bias
        self.grav_gain = grav_gain
        self.grav_offset_deg = grav_offset_deg

    def update(
        self,
        setpoint: float,
        measurement: float,
        measurement_rate: float,
        dt: float,
        flag_setup_feedback_speed: bool,
        param_update: dict | None = None
    ) -> float:

        if param_update:
            self.apply_params(**param_update)

        dt = max(min(dt, 0.05), 1e-3)

        # -------- режим настройки скорости --------
        if flag_setup_feedback_speed:
            output_pi = 0.0
            err_position = 0.0
            setspeed = setpoint
        else:
            # -------- позиционный контур --------
            err_position = normalize_angle_deg(setpoint - measurement)
            stage = err_position * self.K_1
            res_p = stage * self.K_p
            res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
            output_pi = res_p + res_i
            setspeed = 0.0

        # -------- гравитационная компенсация --------
        grav = (
            self.grav_bias
            + math.sin(math.radians(setpoint - self.grav_offset_deg))
            * self.grav_gain
        )

        # -------- ОС по скорости --------
        ap = self.aperiodic_step(measurement_rate, dt)
        feedback_speed = ap * self.K_2

        # -------- итог --------
        error_speed = output_pi + grav + setspeed - feedback_speed

        if self.out_sat is not None:
            out = saturation(error_speed, self.out_sat, -self.out_sat)
        else:
            out = error_speed

        if self.debug_hook is not None:
            self.debug_hook({
                "err_position": err_position,
                "output_pi": output_pi,
                "measurement_rate": measurement_rate,
                "feedback_speed": feedback_speed,
                "grav": grav,
                "error_speed": error_speed,
                "out": out,
            })


        return out

# -----------------------
# Linear controllers
# -----------------------
class DepthController(BaseController):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.prev_depth = 0.0

    def reset(self):
        super().reset()
        self.prev_depth = 0.0

    def update(
        self,
        setpoint: float,
        measurement: float,
        dt: float,
        flag_setup_feedback_speed: bool,
        param_update: dict | None = None
    ) -> float:

        if param_update:
            self.apply_params(**param_update)

        # защита dt
        dt = max(min(dt, 0.05), 1e-3)

        # -------- скорость глубины --------
        depth_rate = (measurement - self.prev_depth) / dt
        self.prev_depth = measurement

        # -------- режим настройки скорости --------
        if flag_setup_feedback_speed:
            output_pi = 0.0
            err_position = 0.0
            setspeed = setpoint
        else:
            # -------- позиционный контур --------
            err_position = setpoint - measurement
            stage = err_position * self.K_1
            res_p = stage * self.K_p
            res_i = self.trapezoidal_integrate(stage * self.K_i, dt)
            output_pi = res_p + res_i
            setspeed = 0.0

        # -------- ОС по скорости --------
        ap = self.aperiodic_step(depth_rate, dt)
        feedback_speed = ap * self.K_2

        # -------- итог --------
        error_speed = output_pi + setspeed - feedback_speed

        if self.out_sat is not None:
            out = saturation(error_speed, self.out_sat, -self.out_sat)
        else:
            out = error_speed

        if self.debug_hook is not None:
            self.debug_hook({
                "err_position": err_position,
                "measurement_rate": depth_rate,
                "feedback_speed": feedback_speed,
                "output_pi": output_pi,
                "out": out,
            })

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

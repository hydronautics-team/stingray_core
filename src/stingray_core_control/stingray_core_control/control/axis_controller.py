# stingray_core_control/control/axis_controller.py

from abc import ABC, abstractmethod


RAD_TO_DEG = 57.3

class AxisController(ABC):
    """
    Абстрактный контроллер одной оси.
    Отвечает за вычисление управляющего воздействия.
    """

    @abstractmethod
    def compute(self, target: float, dt: float) -> float:
        pass

class AngularAxisController(AxisController):
    def __init__(self, controller, angle_fn, rate_fn, feedback_flag_fn):
        self.controller = controller
        self.angle = angle_fn
        self.rate = rate_fn
        self.feedback_flag = feedback_flag_fn

    def compute(self, target, dt):
        rate_deg = self.rate() * RAD_TO_DEG
        return self.controller.update(
            target,
            self.angle(),
            rate_deg,
            dt,
            self.feedback_flag(),
        )

class LinearAxisController(AxisController):
    def __init__(self, controller, pos_fn, vel_fn, accel_fn, feedback_flag_fn):
        self.controller = controller
        self.pos = pos_fn
        self.vel = vel_fn
        self.accel = accel_fn
        self.feedback_flag = feedback_flag_fn

    def compute(self, target, dt):
        return self.controller.update(
            target,
            self.pos(),
            self.vel(),
            self.accel(),
            dt,
            self.feedback_flag(),
        )


class LinearVelocityAxisController(AxisController):
    """
    Контроллер линейной скорости (например, surge),
    где целевая величина — скорость, а обратная связь:
      - скорость
      - ускорение
    """

    def __init__(self, controller, vel_fn, accel_fn):
        self.controller = controller
        self.vel = vel_fn
        self.accel = accel_fn

    def compute(self, target, dt):
        return self.controller.update(
            target,
            self.vel(),
            self.accel(),
            dt,
        )

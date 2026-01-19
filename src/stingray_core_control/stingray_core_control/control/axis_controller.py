# stingray_core_control/control/axis_controller.py

from abc import ABC, abstractmethod

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
        return self.controller.update(
            target,
            self.angle(),
            self.rate(),
            dt,
            self.feedback_flag(),
            param_update=self.controller.__dict__,
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
            param_update=self.controller.__dict__,
        )

import math

from stingray_core_control.control.controllers import SurgeController, SwayController
from stingray_core_control.stingray_core_control_node import advance_position_target


def make_controller(controller_type):
    return controller_type(
        K_p=2.0,
        K_1=3.0,
        K_2=0.5,
        K_i=0.0,
        I_min=-10.0,
        I_max=10.0,
        out_sat=100.0,
        ap_K=1.0,
        ap_T=0.0,
    )


def test_position_error_and_velocity_feedback():
    for controller_type in (SurgeController, SwayController):
        controller = make_controller(controller_type)
        output = controller.update(4.0, 1.0, 2.0, 0.01, False)
        assert math.isclose(output, 17.0)


def test_speed_feedback_setup_mode():
    for controller_type in (SurgeController, SwayController):
        controller = make_controller(controller_type)
        output = controller.update(4.0, 100.0, 2.0, 0.01, True)
        assert math.isclose(output, 3.0)


def test_velocity_command_advances_position_target():
    target = advance_position_target(target=2.0, velocity_command=0.4, dt=0.5)
    assert math.isclose(target, 2.2)


def test_zero_velocity_holds_position_target():
    target = advance_position_target(target=2.0, velocity_command=0.0, dt=0.5)
    assert math.isclose(target, 2.0)

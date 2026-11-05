import math

from stingray_core_control.control.controllers import SurgeController, SwayController
from stingray_core_control.stingray_core_control_node import (
    body_offset_to_odom,
    horizontal_setpoint_changed,
    odom_error_to_body,
    relative_position_target,
    setpoint_changed,
)


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


def test_relative_position_command_uses_current_position():
    target = relative_position_target(current=2.0, offset=0.4)
    assert math.isclose(target, 2.4)


def test_zero_offset_targets_current_position():
    target = relative_position_target(current=2.0, offset=0.0)
    assert math.isclose(target, 2.0)


def test_repeated_streamed_setpoint_is_not_a_new_command():
    assert setpoint_changed(previous=None, current=1.0)
    assert not setpoint_changed(previous=1.0, current=1.0)
    assert setpoint_changed(previous=1.0, current=2.0)


def test_horizontal_body_offset_is_rotated_into_odom():
    offset_x, offset_y = body_offset_to_odom(surge=1.0, sway=0.0, yaw_deg=90.0)
    assert math.isclose(offset_x, 0.0, abs_tol=1e-9)
    assert math.isclose(offset_y, 1.0, abs_tol=1e-9)


def test_horizontal_odom_error_is_rotated_back_to_body():
    surge_error, sway_error = odom_error_to_body(
        error_x=0.0, error_y=1.0, yaw_deg=90.0
    )
    assert math.isclose(surge_error, 1.0, abs_tol=1e-9)
    assert math.isclose(sway_error, 0.0, abs_tol=1e-9)


def test_repeated_horizontal_pair_is_not_a_new_command():
    assert horizontal_setpoint_changed(previous=None, current=(1.0, 2.0))
    assert not horizontal_setpoint_changed(previous=(1.0, 2.0), current=(1.0, 2.0))
    assert horizontal_setpoint_changed(previous=(1.0, 2.0), current=(1.0, 3.0))

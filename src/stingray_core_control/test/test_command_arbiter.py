from stingray_core_control.control.command_arbiter import (
    CommandRoute,
    ControlCommandArbiter,
    ControlMode,
)


def test_manual_mode_accepts_only_manual_commands():
    arbiter = ControlCommandArbiter(ControlMode.MANUAL)

    assert arbiter.route_manual_command() == CommandRoute.MANUAL_IMPACT
    assert arbiter.route_automatic_command() == CommandRoute.IGNORE


def test_assisted_mode_accepts_only_manual_commands():
    arbiter = ControlCommandArbiter(ControlMode.ASSISTED)

    assert arbiter.route_manual_command() == CommandRoute.MANUAL_IMPACT
    assert arbiter.route_automatic_command() == CommandRoute.IGNORE


def test_automatic_mode_accepts_only_automatic_commands():
    arbiter = ControlCommandArbiter(ControlMode.AUTOMATIC)

    assert arbiter.route_manual_command() == CommandRoute.IGNORE
    assert arbiter.route_automatic_command() == CommandRoute.AUTOMATIC_SETPOINT


def test_invalid_mode_is_rejected_without_changing_current_mode():
    arbiter = ControlCommandArbiter(ControlMode.ASSISTED)

    assert not arbiter.set_mode(17)
    assert arbiter.mode == ControlMode.ASSISTED

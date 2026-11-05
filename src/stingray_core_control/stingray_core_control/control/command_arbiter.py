from enum import Enum, IntEnum


class ControlMode(IntEnum):
    MANUAL = 0
    ASSISTED = 1
    AUTOMATIC = 2


class CommandRoute(Enum):
    IGNORE = "ignore"
    MANUAL_IMPACT = "manual_impact"
    AUTOMATIC_SETPOINT = "automatic_setpoint"


class ControlCommandArbiter:
    """Select the command source allowed by the current control mode."""

    def __init__(self, mode: ControlMode = ControlMode.MANUAL):
        self._mode = mode

    @property
    def mode(self) -> ControlMode:
        return self._mode

    def set_mode(self, value: int) -> bool:
        try:
            self._mode = ControlMode(value)
        except ValueError:
            return False
        return True

    def route_manual_command(self) -> CommandRoute:
        if self._mode in (ControlMode.MANUAL, ControlMode.ASSISTED):
            return CommandRoute.MANUAL_IMPACT
        return CommandRoute.IGNORE

    def route_automatic_command(self) -> CommandRoute:
        if self._mode == ControlMode.AUTOMATIC:
            return CommandRoute.AUTOMATIC_SETPOINT
        return CommandRoute.IGNORE

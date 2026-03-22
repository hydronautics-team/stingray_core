# stingray_core_control/state/control.py

from dataclasses import dataclass, field

@dataclass(slots=True)
class ControlState:
    impact: dict[str, float] = field(default_factory=dict)
    enabled: dict[str, bool] = field(default_factory=dict)

    @classmethod
    def from_axes(cls, axes: list[str]) -> "ControlState":
        return cls(
            impact={a: 0.0 for a in axes},
            enabled={a: False for a in axes},
        )

# thruster_mixer.py
from typing import Sequence, Dict, List

class ThrusterMixer:
    """
    Pure mixer: holds thruster names, axes and coeffs.
    coeffs: dict thruster -> list[float] aligned with axes.
    """

    def __init__(self, thrusters: Sequence[str], axes: Sequence[str], coeffs: Dict[str, Sequence[float]]):
        self.thrusters = list(thrusters)
        self.axes = list(axes)
        self.coeffs = {t: [float(x) for x in coeffs.get(t, [0.0]*len(self.axes))] for t in self.thrusters}
        for t in self.thrusters:
            if len(self.coeffs[t]) != len(self.axes):
                raise ValueError(f"coeffs for thruster '{t}' must have length {len(self.axes)}")

    def mix_from_dict(self, control_signals: Dict[str, float]) -> List[int]:
        """control_signals: axis -> value"""
        data: List[int] = []
        for t in self.thrusters:
            s = 0.0
            for i, a in enumerate(self.axes):
                s += self.coeffs[t][i] * float(control_signals.get(a, 0.0))
            val_int = max(100, min(200, int(s + 150)))
            data.append(val_int)
        return data

    def mix_from_list(self, control_list: Sequence[float]) -> List[int]:
        """control_list aligned with self.axes"""
        if len(control_list) != len(self.axes):
            raise ValueError("control_list length must match axes length")
        data: List[int] = []
        for t in self.thrusters:
            s = 0.0
            for i in range(len(self.axes)):
                s += self.coeffs[t][i] * float(control_list[i])
            val_int = max(100, min(200, int(s + 150)))
            data.append(val_int)
        return data

    def update_coeffs(self, coeffs: Dict[str, Sequence[float]]):
        """Replace coefficients at runtime (partial updates allowed)."""
        for t, row in coeffs.items():
            if t in self.coeffs:
                if len(row) != len(self.axes):
                    raise ValueError(f"coeffs for thruster '{t}' must have length {len(self.axes)}")
                self.coeffs[t] = [float(x) for x in row]

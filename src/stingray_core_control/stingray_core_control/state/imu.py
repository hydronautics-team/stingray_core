# stingray_core_control/state/imu.py

from dataclasses import dataclass

@dataclass(slots=True)
class ImuState:
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    rate_x: float = 0.0
    rate_y: float = 0.0
    rate_z: float = 0.0

    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

"""Configuration model for the doctor.

This module deliberately contains no ROS or hardware access. Loading a
platform YAML file can be added without changing any individual check.
"""

from dataclasses import dataclass, field


@dataclass(frozen=True)
class TopicConfig:
    """Expected properties of an already-running topic."""

    name: str
    timeout_sec: float


@dataclass(frozen=True)
class DoctorConfig:
    """Current defaults for the Welt deployment."""

    expected_ros_distro: str = "humble"
    domain_id: int = 1
    require_docker: bool = True
    required_packages: tuple[str, ...] = (
        "stingray_core_communication",
        "stingray_core_control",
        "pressure_sensor",
        "lights_device",
        "dvl_a50",
        "vectornav",
    )
    imu: TopicConfig = field(
        default_factory=lambda: TopicConfig("/vectornav/imu", 3.0)
    )
    pressure: TopicConfig = field(
        default_factory=lambda: TopicConfig(
            "/stingray_core/pressure_sensor/depth", 3.0
        )
    )
    dvl: TopicConfig = field(
        default_factory=lambda: TopicConfig("/dvl/data", 3.0)
    )


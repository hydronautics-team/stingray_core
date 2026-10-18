"""Configuration model for the doctor.

This module deliberately contains no ROS or hardware access. Loading a
platform YAML file can be added without changing any individual check.
"""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any
from warnings import warn

import yaml


class ConfigError(ValueError):
    """Raised when the YAML configuration is missing, malformed, or invalid"""

    def __str__(self) -> str:
        red = "\033[91m"
        reset = "\033[0m"
        original_message = super().__str__()
        return f"{red}{original_message}{reset}"


class ConfigWarn(RuntimeWarning):
    """Raised when the default values from DoctorConfig are used."""

    def __str__(self) -> str:
        yellow = "\033[93m"
        reset = "\033[0m"
        original_message = super().__str__()
        return f"{yellow}{original_message}{reset}"


@dataclass(frozen=True)
class TopicConfig:
    """Expected properties of an already-running topic."""

    name: str
    timeout_sec: float
    expected_frame_id: str | None = None
    min_hz: float | None = None
    max_hz: float | None = None
    min_value: float | None = None
    max_value: float | None = None

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ConfigError("Topic name must be a non-empty string")
        if (
            self.min_hz is not None
            and self.max_hz is not None
            and self.min_hz > self.max_hz
        ):
            raise ConfigError(
                f"min_hz ({self.min_hz}) > max_hz ({self.max_hz}) for '{self.name}'"
            )
        if (
            self.min_value is not None
            and self.max_value is not None
            and self.min_value > self.max_value
        ):
            raise ConfigError(
                f"min_value ({self.min_value}) > max_value ({self.max_value}) for '{self.name}'"
            )


@dataclass(frozen=True)
class DoctorConfig:
    """Current defaults for the Welt deployment."""

    expected_ros_distro: str = "humble"
    expected_ros_domain_id: int = 1
    require_docker: bool = True
    required_packages: tuple[str, ...] = (
        "stingray_core_communication",
        "stingray_core_control",
        "pressure_sensor",
        "lights_device",
        "dvl_a50",
        "vectornav",
    )
    default_timeout_sec: float = 3.0
    window_sec: float = 3.0
    future_timestamp_tolerance_sec: float = 0.1
    imu: TopicConfig = field(
        default_factory=lambda: TopicConfig(
            name="/vectornav/imu",
            timeout_sec=3.0,
            expected_frame_id="vectornav",
            min_hz=20.0,
            max_hz=200.0,
        )
    )
    pressure: TopicConfig = field(
        default_factory=lambda: TopicConfig(
            name="/stingray_core/pressure_sensor/depth",
            timeout_sec=3.0,
            expected_frame_id="dvl_link",
            min_hz=5.0,
            max_hz=50.0,
            min_value=-0.5,
            max_value=100,
        )
    )
    dvl: TopicConfig = field(
        default_factory=lambda: TopicConfig(
            name="/dvl/data",
            timeout_sec=3.0,
            expected_frame_id="dvl_link",
            min_hz=5.0,
            max_hz=50.0,
        )
    )


def _expect_dict(data: Any, context: str) -> dict[str, Any]:
    if not isinstance(data, dict):
        ConfigError(f"'{context}' must be a dictionary")
    return data


def _check_unknown_keys(data: dict[str, Any], allowed: set[str], context: str) -> None:
    unknown = set(data.keys()) - allowed
    if unknown:
        unknown = ", ".join(list(unknown))
        raise ConfigError(f"Unknown fields in '{context}': {unknown}")


def _expect_number(data: Any, context: str) -> float:
    if not (isinstance(data, int) or isinstance(data, float)) and data <= 0:
        raise ConfigError(f"{context} must be a positive float")
    return data


def _parse_topic(
    raw: Any,
    default: TopicConfig,
    default_timeout: float,
    context: str,
) -> TopicConfig:
    if raw is None:
        return TopicConfig(
            name=default.name,
            timeout_sec=default_timeout,
            expected_frame_id=default.expected_frame_id,
            min_hz=default.min_hz,
            max_hz=default.max_hz,
            min_value=default.min_value,
            max_value=default.max_value,
        )

    data = _expect_dict(raw, context)
    allowed_keys = {
        "name",
        "timeout_sec",
        "expected_frame_id",
        "min_hz",
        "max_hz",
        "min_value",
        "max_value",
    }
    _check_unknown_keys(data, allowed_keys, context)

    name = data.get("name", default.name)
    timeout_sec = data["timeout_sec"]
    expected_frame_id = default.expected_frame_id
    min_hz = default.min_hz
    max_hz = default.max_hz
    min_value = default.min_value
    max_value = default.max_value

    if (
        not (isinstance(timeout_sec, int) or isinstance(timeout_sec, float))
        and timeout_sec <= 0
    ):
        raise ConfigError("default_timeout_sec must be a positive float")
    if "expected_frame_id" in data:
        frame_id = data.get("expected_frame_id", default.expected_frame_id)
        if frame_id is not None and (
            not isinstance(frame_id, str) or not frame_id.strip()
        ):
            raise ConfigError(
                f"Field '{context}.expected_frame_id' must be a non-empty string"
            )

    if "min_hz" in data:
        min_hz = _expect_number(data["min_hz"])
    if "max_hz" in data:
        max_hz = _expect_number(data["max_hz"])
    if "min_value" in data:
        min_value = _expect_number(data["min_value"])
    if "max_value" in data:
        max_value = _expect_number(data["max_value"])

    return TopicConfig(
        name=name,
        timeout_sec=timeout_sec,
        expected_frame_id=expected_frame_id,
        min_hz=min_hz,
        max_hz=max_hz,
        min_value=min_value,
        max_value=max_value,
    )


def load_config(path: str | None) -> DoctorConfig:
    """Load DoctorConfig from YAML or return defaults."""
    defaults = DoctorConfig()
    if path is None:
        path_to_default_config = Path(__file__).resolve().parents[4]
        if path_to_default_config is None or not path_to_default_config.is_file():
            warn(
                "\n[WARNING] Default doctor.yaml configuration file not found. "
                "Falling back to default DoctorConfig values.",
                ConfigWarn,
                stacklevel=2,
            )

            return defaults
        file_path = path_to_default_config
    else:
        file_path = Path(path)
    if not file_path.is_file():
        raise ConfigError(f"Configuration file not found: {file_path}")

    try:
        content = file_path.read_text(encoding="utf-8")
        yml = yaml.safe_load(content)
    except yaml.YAMLError as exc:
        raise ConfigError(f"Invalid YAML syntax in '{file_path}': {exc}") from exc

    if yml is None:
        warn(
            "\n[WARNING] YAML configuration file is empty. "
            "Falling back to default DoctorConfig values.",
            ConfigWarn,
            stacklevel=2,
        )
        return defaults

    root = _expect_dict(yml, "root")
    _check_unknown_keys(root, {"doctor"}, "root")
    categories = _expect_dict(root["doctor"], "doctor")
    allowed_categories = {
        "environment",
        "observation",
        "imu",
        "dvl",
        "pressure",
    }
    _check_unknown_keys(categories, allowed_categories, "doctor")

    """Environment"""
    expected_ros_distro = defaults.expected_ros_distro
    expected_ros_domain_id = defaults.expected_ros_domain_id
    require_docker = defaults.require_docker
    required_packages = defaults.required_packages

    if categories.get("environment") is not None:
        env = _expect_dict(categories["environment"], "doctor.environment")
        _check_unknown_keys(
            env,
            {
                "expected_ros_distro",
                "expected_ros_domain_id",
                "require_docker",
                "required_packages",
            },
            "doctor.environment",
        )
        if "expected_ros_distro" in env:
            expected_ros_distro = env["expected_ros_distro"]
            if (
                not isinstance(expected_ros_distro, str)
                or not expected_ros_distro.strip()
            ):
                raise ConfigError("expected_ros_distro must be a non-empty string")
        if "expected_ros_domain_id" in env:
            expected_ros_domain_id = env["expected_ros_domain_id"]
            if (
                not isinstance(expected_ros_domain_id, int)
                and expected_ros_domain_id < 0
            ):
                raise ConfigError(
                    f"expected_ros_domain_id must be a non-negative integer, got {expected_ros_domain_id}"
                )
        if "require_docker" in env:
            require_docker = env["require_docker"]
            if not isinstance(require_docker, bool):
                raise ConfigError(
                    f"require_docker must be a boolean, got {require_docker}"
                )
        if "required_packages" in env:
            pkgs = env["required_packages"]
            if not isinstance(pkgs, list):
                raise ConfigError(
                    "Field 'doctor.environment.required_packages' "
                    "must be a list of non-empty strings"
                )
            required_packages = tuple(pkgs)

    """"Observation"""
    default_timeout_sec = defaults.default_timeout_sec
    window_sec = defaults.window_sec
    future_timestamp_tolerance_sec = defaults.future_timestamp_tolerance_sec
    if categories.get("observation") is not None:
        obs = _expect_dict(categories["observation"], "doctor.observation")
        _check_unknown_keys(
            obs,
            {
                "default_timeout_sec",
                "window_sec",
                "future_timestamp_tolerance_sec",
            },
            "doctor.observation",
        )
        if "default_timeout_sec" in obs:
            default_timeout_sec = obs["default_timeout_sec"]
            if (
                not (
                    isinstance(default_timeout_sec, int)
                    or isinstance(default_timeout_sec, float)
                )
                and default_timeout_sec <= 0
            ):
                raise ConfigError("default_timeout_sec must be a positive float")
        if "window_sec" in obs:
            window_sec = obs["window_sec"]
            if (
                not (isinstance(window_sec, int) or isinstance(window_sec, float))
                and window_sec <= 0
            ):
                raise ConfigError("window_sec")
        if "future_timestamp_tolerance_sec" in obs:
            future_timestamp_tolerance_sec = obs["future_timestamp_tolerance_sec"]
            if (
                not (
                    isinstance(future_timestamp_tolerance_sec, int)
                    or isinstance(future_timestamp_tolerance_sec, float)
                )
                and future_timestamp_tolerance_sec <= 0
            ):
                raise ConfigError("future_timestamp_tolerance_sec")

    """Topics"""
    default_timeout = default_timeout_sec

    imu_cfg = (
        _parse_topic(categories["imu"], defaults.imu, default_timeout, "doctor.imu")
        if "imu" in categories
        else TopicConfig(name=defaults.imu.name, timeout_sec=default_timeout)
    )

    dvl_cfg = (
        _parse_topic(categories["dvl"], defaults.dvl, default_timeout, "doctor.dvl")
        if "dvl" in categories
        else TopicConfig(name=defaults.dvl.name, timeout_sec=default_timeout)
    )

    pressure_cfg = (
        _parse_topic(
            categories["pressure"], defaults.pressure, default_timeout, "doctor.depth"
        )
        if "pressure" in categories
        else TopicConfig(name=defaults.pressure.name, timeout_sec=default_timeout)
    )

    return DoctorConfig(
        expected_ros_distro=expected_ros_distro,
        expected_ros_domain_id=expected_ros_domain_id,
        require_docker=require_docker,
        required_packages=required_packages,
        default_timeout_sec=default_timeout_sec,
        window_sec=window_sec,
        future_timestamp_tolerance_sec=future_timestamp_tolerance_sec,
        imu=imu_cfg,
        pressure=pressure_cfg,
        dvl=dvl_cfg,
    )

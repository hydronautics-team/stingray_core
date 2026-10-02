import os
import yaml
from typing import List, Optional



class DoctorConfig:

    def __init__(self, yaml_path: Optional[str] = None):

        self.expected_ros_distro = "humble"
        self.domain_id = 1
        self.dvl_ip = "192.168.194.95"
        self.cm5_ip = "10.42.0.111"
        self.vectornav_port = "/dev/ttyUSB0"
        self.required_packages = [
            "stingray_core_communication",
            "stingray_core_control",
            "stingray_core_sensors",
            "stingray_core_devices",
        ]

        if yaml_path is not None:
            self.load_from_yaml(yaml_path)

    def load_from_yaml(self, yaml_path: str):

        if not os.path.exists(yaml_path):
            raise FileNotFoundError(f"YAML file not found: {yaml_path}")

        with open(yaml_path, "r", encoding="utf-8") as file:
            data = yaml.safe_load(file)

        if "expected_ros_distro" in data:
            self.expected_ros_distro = data["expected_ros_distro"]
        if "dvl_ip" in data:
            self.dvl_ip = data["dvl_ip"]
        if "cm5_ip" in data:
            self.cm5_ip = data["cm5_ip"]
        if "vectornav_port" in data:
            self.vectornav_port = data["vectornav_port"]

        
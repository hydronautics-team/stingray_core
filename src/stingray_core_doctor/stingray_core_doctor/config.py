import os
import yaml
from typing import Optional
import ast


class DoctorConfig:

    def __init__(self):

        self.launch_path = None # Необходимо указать путь к run_rov.launch.py для определения небходимых пакетов

        self.expected_ros_distro = "humble"
        self.domain_id = 1
        self.dvl_ip = "192.168.194.95"
        self.cm5_ip = "10.42.0.111"
        self.vectornav_port = "/dev/ttyUSB0"
        self.required_packages = [
            "stingray_core_communication",
            "stingray_core_control",
            "pressure_sensor",
            "lights_device",
            "dvl_a50",
            "vectornav"
        ]

        if self.launch_path is not None:
            extracted = self.extract_packages_from_launch(self.launch_path)
            if extracted:
                self.required_packages = extracted


    def extract_packages_from_launch(self, launch_file_path: str) -> list[str]:
        if not os.path.exists(launch_file_path):
            return []

        with open(launch_file_path, "r", encoding="utf-8") as f:
            content = f.read()

        tree = ast.parse(content)
        packages = []

        for node in ast.walk(tree):
            if isinstance(node, ast.Call):
                func_name = ""
                if isinstance(node.func, ast.Name):
                    func_name = node.func.id

                if func_name == "get_package_share_directory" and node.args:
                    first_arg = node.args[0]
                    if isinstance(first_arg, ast.Constant) and isinstance(first_arg.value, str):
                        if first_arg.value not in packages and not first_arg.value.startswith("welt_bringup"):
                            packages.append(first_arg.value)

        return packages

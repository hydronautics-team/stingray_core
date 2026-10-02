import os
from base import BaseCheck
from stingray_core_doctor.types import CheckResult, CheckStatus
from typing import List

class EnvironmentCheck(BaseCheck):
    def __init__(self, config):
        super().__init__(config)
        self.category_name = "Environment"

    def run(self) -> List[CheckResult]:

        results: List[CheckResult] = []

        ros_distro = os.environ.get("ROS_DISTRO")
        expected_ros_distro = self.config.expected_ros_distro

        if ros_distro is None:
            results.append(CheckResult(
                name=f"ROS 2 {expected_ros_distro}",
                status=CheckStatus.FAIL,
                message="ROS 2 environment is not sourced.",
                hint=f"Use command: 'source /opt/ros/{expected_ros_distro}/setup.bash' to source the ROS 2 environment.",
            ))
        elif ros_distro != expected_ros_distro:
            results.append(CheckResult(
                name=f"ROS 2 {expected_ros_distro}",
                status=CheckStatus.FAIL,
                message=f"Found ROS_DISTRO='{ros_distro}', expected '{expected_ros_distro}'",
                hint=f"Use command: 'source /opt/ros/{expected_ros_distro}/setup.bash' to source the correct ROS 2 environment.",
            ))
        else:
            results.append(CheckResult(
                name=f"ROS 2 {expected_ros_distro}",
                status=CheckStatus.OK,
            ))

        ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
        paths = [p for p in ament_prefix.split(":") if p]
        
        

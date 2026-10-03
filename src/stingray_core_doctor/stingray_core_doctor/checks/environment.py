import os
from stingray_core_doctor.checks.base import BaseCheck
from stingray_core_doctor.types import CheckResult, CheckStatus

class EnvironmentCheck(BaseCheck):
    def __init__(self, config):
        super().__init__(config)
        self.category_name = "Environment"

    def run(self) -> list[CheckResult]:

        results: list[CheckResult] = []

        ros_distro = os.environ.get("ROS_DISTRO")
        expected_ros_distro = self.config.expected_ros_distro

        if ros_distro is None:
            results.append(CheckResult(
                name=f"ROS 2 {expected_ros_distro}",
                status=CheckStatus.FAIL,
                message="ROS 2 environment is not sourced.",
                hint=f"Source the correct ROS 2 environment, if you are sure that ROS 2 is installed: 'source /opt/ros/{expected_ros_distro}/setup.bash'",
            ))
        elif ros_distro != expected_ros_distro:
            results.append(CheckResult(
                name=f"ROS 2 {expected_ros_distro}",
                status=CheckStatus.FAIL,
                message=f"Found ROS_DISTRO='{ros_distro}', expected '{expected_ros_distro}'",
                hint=f"Source the correct ROS 2 environment: 'source /opt/ros/{expected_ros_distro}/setup.bash'",
            ))
        else:
            results.append(CheckResult(
                name=f"ROS 2 {expected_ros_distro}",
                status=CheckStatus.OK,
            ))


        ament_prefix = os.environ.get("AMENT_PREFIX_PATH", "")
        paths = [p for p in ament_prefix.split(":") if p]
        workspace = any([p for p in paths if not p.startswith("/opt/ros")])
        
        if workspace:
            results.append(CheckResult(
                name="Workspace",
                status=CheckStatus.OK,
            ))
        else:
            results.append(CheckResult(
                name="Workspace",
                status=CheckStatus.FAIL,
                message="No custom ROS 2 packages found in AMENT_PREFIX_PATH.",
                hint="Ensure that your workspace is built and sourced correctly, use: 'source install/setup.bash'",
            ))
        

        domain_id = os.environ.get("ROS_DOMAIN_ID")
        expected_domain_id = str(self.config.domain_id)

        if domain_id == expected_domain_id:
            results.append(CheckResult(
                name=f"ROS_DOMAIN_ID={expected_domain_id}",
                status=CheckStatus.OK,
            ))
        elif domain_id is None:
            results.append(CheckResult(
                name=f"ROS_DOMAIN_ID=None",
                status=CheckStatus.FAIL,
                message=f"ROS_DOMAIN_ID is not set, expected '{expected_domain_id}'",
                hint=f"Set the ROS_DOMAIN_ID: 'export ROS_DOMAIN_ID={expected_domain_id}'"
            ))
        else:
            results.append(CheckResult(
                name=f"ROS_DOMAIN_ID={domain_id}",
                status=CheckStatus.FAIL,
                message=f"Found ROS_DOMAIN_ID='{domain_id}', expected '{expected_domain_id}'",
                hint=f"Set the ROS_DOMAIN_ID: 'export ROS_DOMAIN_ID={expected_domain_id}'"
            ))

        if os.path.exists("/.dockerenv"):
            results.append(CheckResult(
                name="Docker",
                status=CheckStatus.OK,
            ))
        else:
            results.append(CheckResult(
                name="Docker",
                status=CheckStatus.FAIL,
                message="Not running inside a Docker container.",
                hint="Run the Stingray Core Doctor inside a Docker container.",
            ))

        return results

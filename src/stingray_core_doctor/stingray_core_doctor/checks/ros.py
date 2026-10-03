from stingray_core_doctor.checks.base import BaseCheck
from stingray_core_doctor.types import CheckResult, CheckStatus

class RosCheck(BaseCheck):
    def __init__(self, config):
        super().__init__(config)
        self.category_name = "ROS"

    def run(self) -> list[CheckResult]:
        results = []

        try:
            from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
        except ImportError:
            results.append(CheckResult(
                name="Ament package index",
                status=CheckStatus.FAIL,
                message="ament_index_python is not installed or ROS 2 is not sourced",
                hint="source /opt/ros/humble/setup.bash"
            ))
            return results

        missing_packages = []
        for pkg in self.config.required_packages:
            try:
                get_package_share_directory(pkg)
                results.append(CheckResult(
                    name=f"Package: {pkg}",
                    status=CheckStatus.OK
                ))
            except PackageNotFoundError:
                missing_packages.append(pkg)
                results.append(CheckResult(
                    name=f"Package: {pkg}",
                    status=CheckStatus.FAIL,
                    message=f"Package '{pkg}' not found in workspace",
                    hint="Run 'colcon build' and 'source install/setup.bash'"
                ))

        return results

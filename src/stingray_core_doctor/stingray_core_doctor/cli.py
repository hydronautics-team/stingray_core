"""Command-line interface for read-only pre-flight observations."""

import argparse
import sys

from stingray_core_doctor.checks.environment import EnvironmentCheck
from stingray_core_doctor.checks.ros import RosCheck
from stingray_core_doctor.config import DoctorConfig
from stingray_core_doctor.report import Reporter
from stingray_core_doctor.runner import CheckRunner


def parse_args(argv=None):
    """Parse the mode without importing optional online dependencies."""
    parser = argparse.ArgumentParser(
        description="Read-only pre-flight checks for Stingray Core"
    )
    parser.add_argument(
        "mode",
        nargs="?",
        default="preflight",
        choices=("preflight", "online"),
        help="preflight checks the host; online also observes a running stack",
    )
    return parser.parse_args(argv)


def build_checks(mode, config):
    """Build checks for the selected responsibility boundary."""
    checks = [EnvironmentCheck(config), RosCheck(config)]
    if mode == "online":
        # Keep ROS message dependencies out of host-only preflight mode.
        from stingray_core_doctor.checks.online import OnlineSensorCheck

        checks.append(OnlineSensorCheck(config))
    return checks


def main(argv=None):
    """Run independent observations and return a stable process status."""
    args = parse_args(argv)
    config = DoctorConfig()
    reporter = Reporter()
    reporter.print_header(args.mode)

    categories = CheckRunner().iter_run(build_checks(args.mode, config))
    for category in categories:
        reporter.print_category(category.name, category.results)

    return reporter.print_summary()


if __name__ == "__main__":
    sys.exit(main())

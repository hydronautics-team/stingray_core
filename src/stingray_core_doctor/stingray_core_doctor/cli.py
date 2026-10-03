import sys
from stingray_core_doctor.config import DoctorConfig
from stingray_core_doctor.report import Reporter
from stingray_core_doctor.checks.environment import EnvironmentCheck
from stingray_core_doctor.checks.ros import RosCheck

def main():

    config = DoctorConfig()
    
    reporter = Reporter()
    reporter.print_header()

    checks = [
        EnvironmentCheck(config),
        RosCheck(config),
        ]

    for check in checks:
        results = check.run()
        reporter.print_category(check.category_name, results)

    exit_code = reporter.print_summary()
    sys.exit(exit_code)

if __name__ == "__main__":
    main()

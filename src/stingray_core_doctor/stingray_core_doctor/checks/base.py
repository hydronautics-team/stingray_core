from stingray_core_doctor.types import CheckResult


class BaseCheck:
    def __init__(self, config):
        self.config = config
        self.category_name = "Not_defined"

    def run(self) -> list[CheckResult]:
        raise NotImplementedError("It must implement the run method")

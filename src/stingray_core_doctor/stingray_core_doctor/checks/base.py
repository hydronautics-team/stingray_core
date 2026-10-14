"""Check interface."""

from abc import ABC, abstractmethod

from stingray_core_doctor.types import CheckResult


class BaseCheck(ABC):
    """A read-only group of related observations."""

    category_name = "Uncategorised"

    def __init__(self, config):
        self.config = config

    @abstractmethod
    def run(self) -> list[CheckResult]:
        """Run observations without changing external system state."""

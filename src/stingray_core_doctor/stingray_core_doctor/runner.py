"""Execution boundary for independent checks."""

from dataclasses import dataclass

from stingray_core_doctor.checks.base import BaseCheck
from stingray_core_doctor.types import CheckResult, CheckStatus


@dataclass(frozen=True)
class CheckCategory:
    """Results grouped for reporting."""

    name: str
    results: list[CheckResult]


class CheckRunner:
    """Run every check even when one implementation fails unexpectedly."""

    def iter_run(self, checks: list[BaseCheck]):
        """Yield each category as soon as its check has finished."""
        for check in checks:
            try:
                results = check.run()
            except Exception as error:  # A doctor must still report other checks.
                results = [
                    CheckResult(
                        name="Doctor check execution",
                        status=CheckStatus.FAIL,
                        message=f"{type(error).__name__}: {error}",
                        hint="Run with verbose logging and inspect the check.",
                    )
                ]
            yield CheckCategory(check.category_name, results)

    def run(self, checks: list[BaseCheck]) -> list[CheckCategory]:
        """Collect all categories for programmatic callers and tests."""
        return list(self.iter_run(checks))

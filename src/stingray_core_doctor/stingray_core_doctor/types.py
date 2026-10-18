"""Shared, side-effect-free result types for doctor checks."""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Any


class CheckStatus(IntEnum):
    """Severity returned by a single doctor check."""

    OK = 0
    WARN = 1
    FAIL = 2


@dataclass(frozen=True)
class CheckResult:
    """Result of one observation made by the doctor."""

    name: str
    status: CheckStatus
    message: str = ""
    hint: str | None = None
    metrics: dict[str, Any] = field(default_factory=dict)

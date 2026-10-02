from enum import IntEnum
from typing import Optional

class CheckStatus(IntEnum):
    OK = 0
    WARN = 1
    FAIL = 2

class CheckResult:
    def __init__(self, name: str, status: CheckStatus, message: str = "", hint: Optional[str] = None):
        self.name = name
        self.status = status
        self.message = message
        self.hint = hint

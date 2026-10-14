"""Behavior tests for the side-effect-free doctor core."""

from stingray_core_doctor.checks.base import BaseCheck
from stingray_core_doctor.cli import parse_args
from stingray_core_doctor.config import DoctorConfig
from stingray_core_doctor.runner import CheckRunner
from stingray_core_doctor.types import CheckResult, CheckStatus


class PassingCheck(BaseCheck):
    category_name = "Passing"

    def run(self):
        return [CheckResult("pass", CheckStatus.OK)]


class BrokenCheck(BaseCheck):
    category_name = "Broken"

    def run(self):
        raise RuntimeError("broken on purpose")


def test_default_mode_is_preflight():
    assert parse_args([]).mode == "preflight"


def test_online_mode_is_explicit():
    assert parse_args(["online"]).mode == "online"


def test_runner_isolates_broken_checks():
    config = DoctorConfig()
    categories = CheckRunner().run([BrokenCheck(config), PassingCheck(config)])

    assert categories[0].results[0].status == CheckStatus.FAIL
    assert categories[1].results[0].status == CheckStatus.OK

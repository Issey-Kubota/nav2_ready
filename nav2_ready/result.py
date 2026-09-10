"""Result types and overall status aggregation."""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable


class Status(IntEnum):
    """A diagnostic result ordered by severity."""

    PASS = 0
    WARN = 1
    FAIL = 2


@dataclass(frozen=True)
class CheckResult:
    """One user-facing diagnostic result."""

    check_id: str
    title: str
    status: Status
    observed: str
    causes: tuple[str, ...] = field(default_factory=tuple)
    next_step: str | None = None


def overall_status(results: Iterable[CheckResult]) -> Status:
    """Return the most severe status, or PASS for an empty collection."""

    return max((result.status for result in results), default=Status.PASS)


def exit_code(results: Iterable[CheckResult]) -> int:
    """Map the aggregate result to the public CLI exit code."""

    return int(overall_status(results))

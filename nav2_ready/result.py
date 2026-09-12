"""Result types and overall status aggregation."""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import Iterable


class Status(IntEnum):
    """Represent a diagnostic outcome ordered by severity."""

    PASS = 0
    WARN = 1
    FAIL = 2


@dataclass(frozen=True)
class CheckResult:
    """Store one user-facing diagnostic result.

    Attributes:
        check_id: Stable identifier such as ``TF-001``.
        title: Short human-readable check name.
        status: PASS, WARN, or FAIL outcome.
        observed: Value or condition found during diagnosis.
        causes: Likely explanations for a warning or failure.
        next_step: Suggested investigation step, when applicable.
    """

    check_id: str
    title: str
    status: Status
    observed: str
    causes: tuple[str, ...] = field(default_factory=tuple)
    next_step: str | None = None


def overall_status(results: Iterable[CheckResult]) -> Status:
    """Find the most severe status.

    Args:
        results: Diagnostic results to aggregate.

    Returns:
        The highest severity, or PASS for an empty collection.
    """

    return max((result.status for result in results), default=Status.PASS)


def exit_code(results: Iterable[CheckResult]) -> int:
    """Map diagnostic severity to the public CLI exit code.

    Args:
        results: Diagnostic results to aggregate.

    Returns:
        0 for PASS, 1 for WARN, or 2 for FAIL.
    """

    return int(overall_status(results))

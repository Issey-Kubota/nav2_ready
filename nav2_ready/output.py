"""Human-readable CLI rendering."""

from collections import Counter
from collections.abc import Iterable

from .result import CheckResult, Status, overall_status


def render_report(results: Iterable[CheckResult], distro: str | None, timeout: float) -> str:
    """Render diagnostic results as a terminal-friendly report.

    Args:
        results: Individual diagnostic results in display order.
        distro: Detected ROS distribution, or ``None`` when unavailable.
        timeout: Observation duration in seconds.

    Returns:
        A multiline report containing details and aggregate counts.
    """
    items = list(results)
    lines = [
        "Nav2 Ready v0.1.0",
        f"ROS distribution: {distro or 'unknown'}",
        f"Observation time: {timeout:.1f} sec",
        "",
    ]
    for result in items:
        lines.append(f"[{result.status.name}] {result.check_id} {result.title}")
        lines.append(f"       {result.observed}")
        for cause in result.causes:
            lines.append(f"       Cause: {cause}")
        if result.next_step:
            lines.append(f"       Check: {result.next_step}")
        lines.append("")

    counts = Counter(result.status for result in items)
    lines.append(f"Overall: {overall_status(items).name}")
    lines.append(
        f"{counts[Status.PASS]} passed, {counts[Status.WARN]} warned, "
        f"{counts[Status.FAIL]} failed"
    )
    return "\n".join(lines)

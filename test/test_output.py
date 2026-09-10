from nav2_ready.output import render_report
from nav2_ready.result import CheckResult, Status


def test_report_contains_actionable_details():
    report = render_report(
        [CheckResult(
            "TF-001", "Odometry transform", Status.FAIL,
            "Cannot transform base_link -> odom",
            ("Frames are disconnected.",),
            "Inspect /tf.",
        )],
        "jazzy",
        3.0,
    )
    assert "[FAIL] TF-001" in report
    assert "Cause: Frames are disconnected." in report
    assert "Check: Inspect /tf." in report
    assert "Overall: FAIL" in report
    assert "0 passed, 0 warned, 1 failed" in report

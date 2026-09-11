"""Exercise actual rclpy startup and the installed CLI in an empty ROS domain."""

import os
import subprocess

import pytest


@pytest.mark.parametrize("sim_time", [False, True])
def test_empty_domain_returns_diagnostics(sim_time):
    pytest.importorskip("rclpy")
    command = ["ros2", "run", "nav2_ready", "check", "--timeout", "0.3"]
    if sim_time:
        command += ["--ros-args", "-p", "use_sim_time:=true"]
    env = dict(os.environ, ROS_DOMAIN_ID="173", ROS_LOCALHOST_ONLY="1")
    result = subprocess.run(
        command, env=env, capture_output=True, text=True, timeout=15,
    )
    assert result.returncode == 2, result.stdout + result.stderr
    for check_id in (
        "ENV-001", "TF-001", "ODOM-001", "SENSOR-001", "TF-002",
        "CMD-001", "NAV-001", "TF-003",
    ):
        assert check_id in result.stdout
    assert "Overall: FAIL" in result.stdout
    assert "Traceback" not in result.stderr

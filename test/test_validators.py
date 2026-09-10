from types import SimpleNamespace as NS

from nav2_ready.result import CheckResult, Status, exit_code, overall_status
from nav2_ready.validators import (
    validate_cmd_vel_subscribers,
    validate_distribution,
    validate_laser_scan,
    validate_lifecycle,
    validate_odom_message,
    validate_point_cloud,
    validate_sensor_topic,
)


def odom_message(odom="odom", base="base_link", value=0.0):
    vector = NS(x=value, y=value, z=value)
    orientation = NS(x=value, y=value, z=value, w=1.0)
    pose = NS(position=vector, orientation=orientation)
    twist = NS(linear=vector, angular=vector)
    return NS(
        header=NS(frame_id=odom),
        child_frame_id=base,
        pose=NS(pose=pose),
        twist=NS(twist=twist),
    )


def laser_scan():
    return NS(
        header=NS(frame_id="laser"), ranges=[1.0, float("inf")],
        angle_increment=0.01, range_min=0.1, range_max=10.0,
    )


def point_cloud(fields=("x", "y", "z")):
    return NS(
        header=NS(frame_id="lidar"), width=2, height=1, data=b"12345678",
        point_step=4, fields=[NS(name=name) for name in fields],
    )


def test_distribution_statuses():
    assert validate_distribution("jazzy").status == Status.PASS
    assert validate_distribution("kilted").status == Status.WARN
    assert validate_distribution(None).status == Status.FAIL


def test_valid_odom_passes():
    result = validate_odom_message(odom_message(), "odom", "base_link")
    assert result.status == Status.PASS


def test_odom_frame_mismatch_fails():
    result = validate_odom_message(odom_message(odom="world"), "odom", "base_link")
    assert result.status == Status.FAIL
    assert "world" in result.observed


def test_odom_nan_fails():
    assert validate_odom_message(
        odom_message(value=float("nan")), "odom", "base_link",
    ).status == Status.FAIL


def test_laser_scan_allows_infinity_ranges():
    assert validate_laser_scan(laser_scan()).status == Status.PASS


def test_empty_laser_scan_fails():
    message = laser_scan()
    message.ranges = []
    assert validate_laser_scan(message).status == Status.FAIL


def test_valid_point_cloud_passes():
    assert validate_point_cloud(point_cloud()).status == Status.PASS


def test_point_cloud_requires_xyz():
    result = validate_point_cloud(point_cloud(("x", "z")))
    assert result.status == Status.FAIL
    assert "y" in result.observed


def test_sensor_topic_supports_scan_and_cloud():
    assert validate_sensor_topic("/scan", ["sensor_msgs/msg/LaserScan"]) is None
    assert validate_sensor_topic("/points", ["sensor_msgs/msg/PointCloud2"]) is None
    result = validate_sensor_topic("/image", ["sensor_msgs/msg/Image"])
    assert result is not None and result.status == Status.FAIL


def test_cmd_vel_statuses():
    assert validate_cmd_vel_subscribers("/cmd_vel", []).status == Status.FAIL
    assert validate_cmd_vel_subscribers(
        "/cmd_vel", ["geometry_msgs/msg/Twist"],
    ).status == Status.PASS
    assert validate_cmd_vel_subscribers(
        "/cmd_vel",
        ["geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"],
    ).status == Status.WARN


def test_lifecycle_statuses():
    assert validate_lifecycle(None).status == Status.WARN
    assert validate_lifecycle({"planner_server": "active"}).status == Status.PASS
    assert validate_lifecycle({"planner_server": "inactive"}).status == Status.FAIL


def test_overall_status_and_exit_code():
    results = [
        CheckResult("A", "a", Status.PASS, "ok"),
        CheckResult("B", "b", Status.WARN, "maybe"),
    ]
    assert overall_status(results) == Status.WARN
    assert exit_code(results) == 1

"""Pure validation functions used by the ROS adapters and unit tests."""

import math
from collections.abc import Iterable, Mapping
from typing import Any

from .result import CheckResult, Status


def validate_distribution(distro: str | None) -> CheckResult:
    if distro == "jazzy":
        return CheckResult("ENV-001", "ROS 2 distribution", Status.PASS, "Detected: jazzy")
    if not distro:
        return CheckResult(
            "ENV-001",
            "ROS 2 distribution",
            Status.FAIL,
            "ROS_DISTRO is not set",
            ("The ROS 2 environment may not be sourced.",),
            "Run: source /opt/ros/jazzy/setup.bash",
        )
    return CheckResult(
        "ENV-001",
        "ROS 2 distribution",
        Status.WARN,
        f"Detected: {distro}; supported: jazzy",
        ("This ROS distribution has not been validated for Nav2 Ready v0.1.",),
        "Re-run in ROS 2 Jazzy before reporting a compatibility issue.",
    )


def validate_odom_message(message: Any, odom_frame: str, base_frame: str) -> CheckResult:
    if message is None:
        return CheckResult(
            "ODOM-001", "Odometry stream", Status.FAIL, "No Odometry message received",
            ("The publisher may be stopped or QoS may be incompatible.",),
            "Check the odometry publisher and topic remapping.",
        )

    actual_odom = message.header.frame_id
    actual_base = message.child_frame_id
    if actual_odom != odom_frame or actual_base != base_frame:
        return CheckResult(
            "ODOM-001", "Odometry stream", Status.FAIL,
            f"Frames: {actual_odom or '<empty>'} -> {actual_base or '<empty>'}",
            (f"Expected {odom_frame} -> {base_frame}.",),
            "Align the Odometry frame IDs with the Nav2 frame parameters.",
        )

    values = (
        message.pose.pose.position.x,
        message.pose.pose.position.y,
        message.pose.pose.position.z,
        message.pose.pose.orientation.x,
        message.pose.pose.orientation.y,
        message.pose.pose.orientation.z,
        message.pose.pose.orientation.w,
        message.twist.twist.linear.x,
        message.twist.twist.linear.y,
        message.twist.twist.angular.z,
    )
    if not all(math.isfinite(value) for value in values):
        return CheckResult(
            "ODOM-001", "Odometry stream", Status.FAIL,
            "Odometry contains NaN or infinity",
            ("The state estimator or sensor input produced a non-finite value.",),
            "Inspect the odometry/state-estimation output before starting Nav2.",
        )
    return CheckResult(
        "ODOM-001", "Odometry stream", Status.PASS,
        f"Received nav_msgs/msg/Odometry ({actual_odom} -> {actual_base})",
    )


def validate_laser_scan(message: Any) -> CheckResult:
    if message is None:
        return _missing_sensor("sensor_msgs/msg/LaserScan")
    if not message.header.frame_id:
        return _bad_sensor("LaserScan frame_id is empty")
    if not message.ranges:
        return _bad_sensor("LaserScan ranges is empty")
    if message.angle_increment <= 0.0:
        return _bad_sensor("LaserScan angle_increment must be greater than zero")
    if message.range_min >= message.range_max:
        return _bad_sensor("LaserScan range_min must be less than range_max")
    return CheckResult(
        "SENSOR-001", "Obstacle sensor stream", Status.PASS,
        f"Received sensor_msgs/msg/LaserScan (frame: {message.header.frame_id})",
    )


def validate_point_cloud(message: Any) -> CheckResult:
    if message is None:
        return _missing_sensor("sensor_msgs/msg/PointCloud2")
    if not message.header.frame_id:
        return _bad_sensor("PointCloud2 frame_id is empty")
    if message.width * message.height <= 0 or not message.data:
        return _bad_sensor("PointCloud2 contains no points")
    if message.point_step <= 0:
        return _bad_sensor("PointCloud2 point_step must be greater than zero")
    fields = {field.name for field in message.fields}
    missing = sorted({"x", "y", "z"} - fields)
    if missing:
        return _bad_sensor(f"PointCloud2 is missing fields: {', '.join(missing)}")
    return CheckResult(
        "SENSOR-001", "Obstacle sensor stream", Status.PASS,
        f"Received sensor_msgs/msg/PointCloud2 (frame: {message.header.frame_id})",
    )


def validate_sensor_topic(topic: str, topic_types: Iterable[str]) -> CheckResult | None:
    types = tuple(topic_types)
    if not types:
        return CheckResult(
            "SENSOR-001", "Obstacle sensor stream", Status.FAIL,
            f"Topic not found: {topic}",
            ("The sensor driver may not be running or the topic name may differ.",),
            "Pass the actual topic with --sensor-topic.",
        )
    supported = {"sensor_msgs/msg/LaserScan", "sensor_msgs/msg/PointCloud2"}
    if not supported.intersection(types):
        return CheckResult(
            "SENSOR-001", "Obstacle sensor stream", Status.FAIL,
            f"Unsupported type on {topic}: {', '.join(types)}",
            ("v0.1 supports LaserScan and PointCloud2 only.",),
            "Use a supported Nav2 obstacle source or convert the sensor data.",
        )
    return None


def validate_cmd_vel_subscribers(topic: str, types: Iterable[str]) -> CheckResult:
    found = set(types)
    supported = {"geometry_msgs/msg/Twist", "geometry_msgs/msg/TwistStamped"}
    compatible = found & supported
    if not found:
        return CheckResult(
            "CMD-001", "Velocity command input", Status.FAIL,
            f"No subscriber found on {topic}",
            ("The base controller may not be running or uses another topic.",),
            "Start the base controller or pass its topic with --cmd-vel-topic.",
        )
    if not compatible:
        return CheckResult(
            "CMD-001", "Velocity command input", Status.FAIL,
            f"Unsupported subscriber type: {', '.join(sorted(found))}",
            ("Expected Twist or TwistStamped.",),
            "Align the base controller and Nav2 cmd_vel message types.",
        )
    if len(compatible) > 1:
        return CheckResult(
            "CMD-001", "Velocity command input", Status.WARN,
            "Both Twist and TwistStamped subscribers were found",
            ("The active Nav2 output type may be ambiguous.",),
            "Verify enable_stamped_cmd_vel across Nav2 and the base controller.",
        )
    return CheckResult(
        "CMD-001", "Velocity command input", Status.PASS,
        f"Compatible subscriber found: {next(iter(compatible))}",
    )


def validate_lifecycle(states: Mapping[str, str] | None) -> CheckResult:
    if not states:
        return CheckResult(
            "NAV-001", "Nav2 lifecycle", Status.WARN,
            "Nav2 lifecycle nodes were not detected",
            ("Nav2 may not be running yet.",),
            "Start Nav2 to perform the runtime readiness checks.",
        )
    bad = {name: state for name, state in states.items() if state != "active"}
    if bad:
        details = ", ".join(f"{name}={state}" for name, state in sorted(bad.items()))
        return CheckResult(
            "NAV-001", "Nav2 lifecycle", Status.FAIL, details,
            ("One or more required Nav2 nodes are not active.",),
            "Inspect the lifecycle manager and the affected node logs.",
        )
    return CheckResult(
        "NAV-001", "Nav2 lifecycle", Status.PASS,
        "Required nodes active: " + ", ".join(sorted(states)),
    )


def _missing_sensor(expected_type: str) -> CheckResult:
    return CheckResult(
        "SENSOR-001", "Obstacle sensor stream", Status.FAIL,
        f"No {expected_type} message received",
        ("The publisher may be stopped or QoS may be incompatible.",),
        "Check the sensor driver, topic remapping, and QoS.",
    )


def _bad_sensor(observed: str) -> CheckResult:
    return CheckResult(
        "SENSOR-001", "Obstacle sensor stream", Status.FAIL, observed,
        ("The sensor message does not meet the minimum Nav2 input contract.",),
        "Inspect the sensor driver output.",
    )

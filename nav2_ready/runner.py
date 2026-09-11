"""ROS 2 adapters and orchestration for the v0.1 checks."""

from dataclasses import dataclass
import os
import time

from .result import CheckResult, Status
from .validators import (
    validate_cmd_vel_subscribers,
    validate_distribution,
    validate_laser_scan,
    validate_lifecycle,
    validate_odom_message,
    validate_point_cloud,
    validate_sensor_topic,
)


@dataclass(frozen=True)
class Config:
    """CLI-adjustable names; defaults match a conventional Nav2 setup."""

    odom_topic: str = "/odom"
    sensor_topic: str = "/scan"
    cmd_vel_topic: str = "/cmd_vel"
    base_frame: str = "base_link"
    odom_frame: str = "odom"
    map_frame: str = "map"
    namespace: str = "/"
    timeout: float = 3.0


CORE_LIFECYCLE_NODES = (
    "controller_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
)


def run_checks(config: Config, ros_args=None) -> tuple[list[CheckResult], str | None]:
    """Run all checks inside one short-lived rclpy node."""

    import rclpy
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2
    from tf2_ros import Buffer, TransformException, TransformListener

    class DiagnosticNode(Node):
        def __init__(self) -> None:
            super().__init__("nav2_ready")
            self.odom_message = None
            self.sensor_message = None
            self.sensor_type = None
            self.tf_buffer = Buffer(node=self)
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.diagnostic_subscriptions = []

    distro = os.environ.get("ROS_DISTRO")
    rclpy.init(args=ros_args)
    node = DiagnosticNode()
    try:
        odom_types = []
        sensor_types = []
        odom_subscribed = False
        odom_stamps = set()

        def receive_odom(message):
            node.odom_message = message
            stamp = message.header.stamp
            odom_stamps.add((stamp.sec, stamp.nanosec))

        deadline = time.monotonic() + config.timeout
        while time.monotonic() < deadline:
            topic_map = dict(node.get_topic_names_and_types())
            odom_types = topic_map.get(config.odom_topic, [])
            sensor_types = topic_map.get(config.sensor_topic, [])
            if not odom_subscribed and "nav_msgs/msg/Odometry" in odom_types:
                node.diagnostic_subscriptions.append(node.create_subscription(
                    Odometry, config.odom_topic, receive_odom,
                    qos_profile_sensor_data,
                ))
                odom_subscribed = True
            if node.sensor_type is None:
                for type_name, message_type in (
                    ("sensor_msgs/msg/LaserScan", LaserScan),
                    ("sensor_msgs/msg/PointCloud2", PointCloud2),
                ):
                    if type_name in sensor_types:
                        node.sensor_type = type_name
                        node.diagnostic_subscriptions.append(node.create_subscription(
                            message_type, config.sensor_topic,
                            lambda message: setattr(node, "sensor_message", message),
                            qos_profile_sensor_data,
                        ))
                        break
            rclpy.spin_once(node, timeout_sec=min(
                0.05, max(0.0, deadline - time.monotonic()),
            ))

        sensor_topic_result = validate_sensor_topic(config.sensor_topic, sensor_types)
        results = [validate_distribution(distro)]
        results.append(_check_transform(
            node, config.odom_frame, config.base_frame, "TF-001",
            "Odometry transform", TransformException, require_fresh=True,
        ))

        if not odom_types:
            results.append(CheckResult(
                "ODOM-001", "Odometry stream", Status.FAIL,
                f"Topic not found: {config.odom_topic}",
                ("The odometry publisher may not be running.",),
                "Start the odometry publisher or pass --odom-topic.",
            ))
        elif "nav_msgs/msg/Odometry" not in odom_types:
            results.append(CheckResult(
                "ODOM-001", "Odometry stream", Status.FAIL,
                f"Unexpected type: {', '.join(odom_types)}",
                ("Expected nav_msgs/msg/Odometry.",),
                "Correct the publisher type or topic selection.",
            ))
        else:
            odom_result = validate_odom_message(
                node.odom_message, config.odom_frame, config.base_frame,
            )
            if odom_result.status == Status.PASS:
                stamp = node.odom_message.header.stamp
                age = (node.get_clock().now().nanoseconds / 1e9
                       - stamp.sec - stamp.nanosec / 1e9)
                if age > 1.0:
                    odom_result = CheckResult(
                        "ODOM-001", "Odometry stream", Status.FAIL,
                        f"Odometry timestamp is stale ({age:.2f} sec old)",
                        ("The publisher may have stopped updating timestamps.",),
                        "Inspect odometry publication and ROS time settings.",
                    )
                elif age < -0.1 or len(odom_stamps) < 2:
                    odom_result = CheckResult(
                        "ODOM-001", "Odometry stream", Status.WARN,
                        "Odometry time is ahead or timestamp progression was not observed",
                        ("The observation may be too short, or ROS clocks disagree.",),
                        "Check ROS time settings and repeat with a longer --timeout.",
                    )
            results.append(odom_result)

        if sensor_topic_result is not None:
            results.append(sensor_topic_result)
            sensor_frame = None
        elif node.sensor_type == "sensor_msgs/msg/LaserScan":
            results.append(validate_laser_scan(node.sensor_message))
            sensor_frame = getattr(
                getattr(node.sensor_message, "header", None), "frame_id", None,
            )
        else:
            results.append(validate_point_cloud(node.sensor_message))
            sensor_frame = getattr(
                getattr(node.sensor_message, "header", None), "frame_id", None,
            )

        if sensor_frame:
            results.append(_check_transform(
                node, config.base_frame, sensor_frame, "TF-002",
                "Sensor transform", TransformException,
            ))
        else:
            results.append(CheckResult(
                "TF-002", "Sensor transform", Status.FAIL,
                "Sensor frame is unavailable",
                ("A valid sensor message is required to discover its frame.",),
                "Resolve SENSOR-001 first.",
            ))

        subscriber_types = [
            endpoint.topic_type
            for endpoint in node.get_subscriptions_info_by_topic(config.cmd_vel_topic)
        ]
        results.append(validate_cmd_vel_subscribers(
            config.cmd_vel_topic, subscriber_types,
        ))

        lifecycle_states = _get_lifecycle_states(
            node, config, GetState, rclpy,
        )
        lifecycle_result = validate_lifecycle(lifecycle_states)
        results.append(lifecycle_result)

        if lifecycle_result.status == Status.WARN:
            results.append(CheckResult(
                "TF-003", "Global localization transform", Status.WARN,
                f"{config.map_frame} -> {config.odom_frame} was not evaluated",
                ("Nav2 lifecycle nodes were not detected.",),
                "Start Nav2 localization or SLAM and run the check again.",
            ))
        else:
            results.append(_check_transform(
                node, config.map_frame, config.odom_frame, "TF-003",
                "Global localization transform", TransformException,
                require_fresh=True,
            ))
        return results, distro
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _check_transform(node, target: str, source: str, check_id: str,
                     title: str, transform_exception,
                     require_fresh: bool = False) -> CheckResult:
    from rclpy.time import Time

    try:
        transform = node.tf_buffer.lookup_transform(target, source, Time())
    except transform_exception as exc:
        return CheckResult(
            check_id, title, Status.FAIL,
            f"Cannot transform {source} -> {target}: {exc}",
            ("The frames may be missing, disconnected, or named differently.",),
            "Inspect /tf, /tf_static, URDF, and the relevant frame parameters.",
        )
    if require_fresh:
        stamp = Time.from_msg(transform.header.stamp)
        if stamp.nanoseconds == 0:
            return CheckResult(
                check_id, title, Status.FAIL,
                f"Transform {source} -> {target} has a zero timestamp",
                ("A dynamic Nav2 transform may have been published as static.",),
                "Publish this transform continuously from localization or odometry.",
            )
        age = (node.get_clock().now() - stamp).nanoseconds / 1_000_000_000
        if age > 1.0:
            return CheckResult(
                check_id, title, Status.FAIL,
                f"Transform {source} -> {target} is stale ({age:.2f} sec old)",
                ("The transform broadcaster may have stopped updating.",),
                "Inspect the broadcaster and ROS time configuration.",
            )
        if age < -0.1:
            return CheckResult(
                check_id, title, Status.WARN,
                f"Transform timestamp is {-age:.2f} sec ahead of diagnostic time",
                ("Nodes may disagree about use_sim_time or the system clock.",),
                "Align ROS time settings across the robot and Nav2 nodes.",
            )
    return CheckResult(
        check_id, title, Status.PASS, f"Transform available: {source} -> {target}",
    )


def _get_lifecycle_states(node, config: Config, get_state_type, rclpy):
    service_names = {name for name, _ in node.get_service_names_and_types()}
    paths = {
        name: _namespaced(config.namespace, name, "get_state")
        for name in CORE_LIFECYCLE_NODES
    }
    if not any(path in service_names for path in paths.values()):
        return None

    states = {}
    for name, path in paths.items():
        if path not in service_names:
            states[name] = "missing"
            continue
        client = node.create_client(get_state_type, path)
        if not client.wait_for_service(timeout_sec=0.2):
            states[name] = "unreachable"
            continue
        future = client.call_async(get_state_type.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
        response = future.result() if future.done() else None
        states[name] = response.current_state.label if response else "no response"
    return states


def _namespaced(namespace: str, node_name: str, service: str) -> str:
    prefix = namespace.strip("/")
    parts = [part for part in (prefix, node_name, service) if part]
    return "/" + "/".join(parts)

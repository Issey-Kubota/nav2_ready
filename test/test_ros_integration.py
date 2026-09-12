"""Exercise the installed CLI against a synthetic ROS graph, without motion."""

import os
import subprocess
import sys
import threading
import time

import pytest


@pytest.mark.parametrize("scenario,expected", [
    ("scan", None),
    ("cloud", None),
    ("future_global_tf", None),
    ("missing_sensor_tf", "[FAIL] TF-002"),
    ("missing_odom_tf", "[FAIL] TF-001"),
    ("wrong_odom_frame", "[FAIL] ODOM-001"),
    ("frozen_odom", "[FAIL] ODOM-001"),
    ("inactive", "[FAIL] NAV-001"),
    ("missing_cmd", "[FAIL] CMD-001"),
    ("no_nav", "[WARN] NAV-001"),
])
def test_robot_graph(scenario, expected):
    pytest.importorskip("rclpy")
    scenarios = (
        "scan", "cloud", "future_global_tf", "missing_sensor_tf",
        "missing_odom_tf", "wrong_odom_frame", "frozen_odom", "inactive",
        "missing_cmd", "no_nav",
    )
    # DDS graph cleanup is asynchronous. A distinct domain per scenario avoids
    # endpoints from the preceding subprocess affecting discovery.
    env = dict(
        os.environ,
        ROS_DOMAIN_ID=str(174 + scenarios.index(scenario)),
        ROS_LOCALHOST_ONLY="1",
    )
    robot = subprocess.Popen(
        [sys.executable, __file__, scenario], env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
    )
    ready = threading.Event()
    logs = []

    def collect():
        for line in robot.stdout:
            logs.append(line)
            if "ROBOT_READY" in line:
                ready.set()

    reader = threading.Thread(target=collect, daemon=True)
    reader.start()
    try:
        assert ready.wait(10), "".join(logs)
        # Give discovery time to propagate between the two fresh participants;
        # loaded CI runners can take longer than a local ROS graph.
        time.sleep(1.0)
        assert robot.poll() is None, "".join(logs)
        result = subprocess.run(
            ["ros2", "run", "nav2_ready", "check", "--timeout", "5"],
            env=env, capture_output=True, text=True, timeout=20,
        )
        report = result.stdout + result.stderr
        assert "Traceback" not in report, report
        if expected is None:
            assert result.returncode == 0, report
            assert "8 passed, 0 warned, 0 failed" in report, report
        else:
            assert expected in report, report
            assert result.returncode == (1 if scenario == "no_nav" else 2), report
    finally:
        robot.terminate()
        try:
            robot.wait(timeout=5)
        except subprocess.TimeoutExpired:
            robot.kill()
            robot.wait(timeout=5)
        reader.join(timeout=2)


def run_robot(scenario):
    import struct

    import rclpy
    from geometry_msgs.msg import TransformStamped, Twist
    from lifecycle_msgs.srv import GetState
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan, PointCloud2, PointField
    from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

    rclpy.init(args=[])
    node = Node("synthetic_robot")
    dynamic = TransformBroadcaster(node)
    static = StaticTransformBroadcaster(node)
    odom_pub = node.create_publisher(Odometry, "/odom", 10)
    sensor_cls = PointCloud2 if scenario == "cloud" else LaserScan
    sensor_pub = node.create_publisher(sensor_cls, "/scan", 10)
    if scenario != "missing_cmd":
        node.create_subscription(Twist, "/cmd_vel", lambda msg: None, 10)

    def state(request, response):
        response.current_state.id = 2 if scenario == "inactive" else 3
        response.current_state.label = "inactive" if scenario == "inactive" else "active"
        return response

    if scenario != "no_nav":
        for name in ("controller_server", "planner_server", "behavior_server", "bt_navigator"):
            node.create_service(GetState, f"/{name}/get_state", state)

    def transform(parent, child, future_seconds=0.0):
        msg = TransformStamped()
        msg.header.frame_id = parent
        msg.child_frame_id = child
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.stamp.nanosec += int(future_seconds * 1_000_000_000)
        if msg.header.stamp.nanosec >= 1_000_000_000:
            msg.header.stamp.sec += 1
            msg.header.stamp.nanosec -= 1_000_000_000
        msg.transform.rotation.w = 1.0
        return msg

    if scenario != "missing_sensor_tf":
        static.sendTransform(transform("base_link", "laser"))
    frozen_stamp = node.get_clock().now().to_msg()
    frozen_stamp.sec -= 10

    def publish():
        if scenario != "missing_sensor_tf":
            static.sendTransform(transform("base_link", "laser"))
        transforms = [transform(
            "map", "odom", 0.8 if scenario == "future_global_tf" else 0.0,
        )]
        if scenario != "missing_odom_tf":
            transforms.append(transform("odom", "base_link"))
        dynamic.sendTransform(transforms)
        odom = Odometry()
        odom.header.frame_id = "wrong" if scenario == "wrong_odom_frame" else "odom"
        odom.child_frame_id = "base_link"
        odom.header.stamp = (frozen_stamp if scenario == "frozen_odom"
                             else node.get_clock().now().to_msg())
        odom.pose.pose.orientation.w = 1.0
        odom_pub.publish(odom)
        sensor = sensor_cls()
        sensor.header.frame_id = "laser"
        sensor.header.stamp = node.get_clock().now().to_msg()
        if scenario == "cloud":
            sensor.height, sensor.width = 1, 1
            sensor.fields = [PointField(name=name, offset=i * 4,
                                        datatype=PointField.FLOAT32, count=1)
                             for i, name in enumerate(("x", "y", "z"))]
            sensor.point_step = sensor.row_step = 12
            sensor.data = struct.pack("<fff", 1.0, 0.0, 0.0)
        else:
            sensor.angle_increment = 0.1
            sensor.range_min, sensor.range_max = 0.1, 10.0
            sensor.ranges = [1.0, 2.0, 3.0]
        sensor_pub.publish(sensor)

    node.create_timer(0.05, publish)
    print("ROBOT_READY", flush=True)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    run_robot(sys.argv[1])

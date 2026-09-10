from nav2_ready.cli import build_parser


def test_cli_defaults():
    args = build_parser().parse_args([])
    assert args.odom_topic == "/odom"
    assert args.sensor_topic == "/scan"
    assert args.cmd_vel_topic == "/cmd_vel"
    assert args.base_frame == "base_link"
    assert args.timeout == 3.0


def test_cli_point_cloud_topic_override():
    args = build_parser().parse_args(["--sensor-topic", "/points"])
    assert args.sensor_topic == "/points"

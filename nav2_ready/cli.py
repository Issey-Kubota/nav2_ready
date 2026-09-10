"""Command-line entry point."""

import argparse
import sys

from .output import render_report
from .result import exit_code
from .runner import Config, run_checks


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Check whether a custom robot meets basic Nav2 prerequisites.",
    )
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--sensor-topic", default="/scan")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--odom-frame", default="odom")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--namespace", default="/")
    parser.add_argument("--timeout", type=float, default=3.0)
    return parser


def main(argv=None) -> int:
    args, ros_args = build_parser().parse_known_args(argv)
    if ros_args and "--ros-args" not in ros_args:
        print(
            "nav2_ready: unrecognized arguments: " + " ".join(ros_args),
            file=sys.stderr,
        )
        return 3
    if args.timeout <= 0:
        print("nav2_ready: --timeout must be greater than zero", file=sys.stderr)
        return 3
    config = Config(**vars(args))
    try:
        results, distro = run_checks(config, ros_args or None)
    except (ImportError, RuntimeError) as exc:
        print(f"nav2_ready: unable to run diagnostics: {exc}", file=sys.stderr)
        return 3
    print(render_report(results, distro, config.timeout))
    return exit_code(results)


if __name__ == "__main__":
    raise SystemExit(main())

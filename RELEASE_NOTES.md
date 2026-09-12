# Release notes

## v0.1.0 (draft)

Nav2 Ready v0.1.0 is the first public release of a read-only CLI for checking
whether a custom ROS 2 robot exposes the basic interfaces expected by Nav2.

### Highlights

- Reports each check as `PASS`, `WARN`, or `FAIL`.
- Shows the observed condition, likely cause, and suggested next step.
- Reads the ROS graph and messages without publishing velocity commands or
  changing lifecycle states.
- Supports one `sensor_msgs/msg/LaserScan` or
  `sensor_msgs/msg/PointCloud2` obstacle source per run.
- Accepts topic, frame, namespace, and observation-time overrides.
- Returns stable exit codes suitable for scripts and CI.

### Checks included

| ID | Check |
| --- | --- |
| `ENV-001` | ROS 2 distribution |
| `TF-001` | `odom` to robot base transform and freshness |
| `ODOM-001` | Odometry topic, frames, values, and timestamp progression |
| `SENSOR-001` | LaserScan or PointCloud2 obstacle input |
| `TF-002` | Sensor frame to robot base transform |
| `CMD-001` | Compatible velocity-command subscriber |
| `NAV-001` | Core Nav2 lifecycle node states |
| `TF-003` | `map` to `odom` localization transform and freshness |

### Supported environment

- ROS 2 Jazzy on Ubuntu 24.04, including a Jazzy Docker container.
- Custom topic and frame names through CLI options.
- ROS simulation time through standard ROS arguments.

### Validation

- Unit and CLI tests run on ROS 2 Jazzy in GitHub Actions.
- Synthetic ROS graph tests cover normal LaserScan and PointCloud2 setups and
  representative TF, Odometry, lifecycle, sensor, and command-input failures.
- A complete PASS result was manually verified with the Jazzy TurtleBot 3 Nav2
  simulation.

### Known limitations

- Only ROS 2 Jazzy is validated.
- Exactly one obstacle sensor topic is checked per run.
- PointCloud2 validation checks structure, not point-cloud quality or Nav2
  voxel-layer configuration.
- Nav2 tuning, complete URDF validation, costmap configuration, navigation
  performance, rosbag analysis, and automatic fixes are outside v0.1 scope.
- A PASS result does not certify navigation correctness or robot safety.

### Install and run

Follow the source-build instructions in the
[README](https://github.com/Issey-Kubota/nav2_ready#build), start the robot
interfaces, and run:

```bash
ros2 run nav2_ready check
```

Please report incorrect diagnostics through the bug-report form. Usage
questions and real custom-robot integration experiences are welcome through
the feedback form.

# Nav2 Ready

Nav2 Ready is a read-only CLI that checks whether a custom ROS 2 robot meets
the basic interface requirements for Nav2. It reports `PASS`, `WARN`, or `FAIL`
with the observed value and a concrete next step.

Version 0.1 targets ROS 2 Jazzy and intentionally keeps the scope small.

Development snapshot, not a released package. ROS 2 Jazzy builds and synthetic
topic/TF/lifecycle integration tests run in CI. PASS does not certify navigation
correctness or robot safety. Please report questions through GitHub Issues.

## Checks

- ROS distribution
- `odom -> base_link` transform
- `nav_msgs/msg/Odometry` stream and frame IDs
- one obstacle sensor stream: `LaserScan` or `PointCloud2`
- sensor frame transform to `base_link`
- compatible `cmd_vel` subscriber
- core Nav2 lifecycle node states
- `map -> odom` transform when Nav2 is running

Nav2 Ready never publishes a velocity command and cannot move the robot.

## Build

```bash
source /opt/ros/jazzy/setup.bash
mkdir -p ~/nav2_ready_ws/src
cd ~/nav2_ready_ws/src
git clone https://github.com/Issey-Kubota/nav2_ready.git nav2_ready
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Run

Start the robot drivers, state publisher, odometry, and obstacle sensor first.
Nav2 may be started before or after the first diagnostic run.

```bash
ros2 run nav2_ready check
```

For a 3D LiDAR or a non-standard topic:

```bash
ros2 run nav2_ready check --sensor-topic /points
```

When using simulation time, pass the standard ROS parameter:

```bash
ros2 run nav2_ready check --ros-args -p use_sim_time:=true
```

Match topic and frame overrides to the robot's Nav2 parameters. For example,
the Jazzy TurtleBot 3 simulation uses `base_footprint`:

```bash
ros2 run nav2_ready check --base-frame base_footprint \
  --ros-args -p use_sim_time:=true
```

Use `--help` to see topic, frame, namespace, and timeout overrides.

## Exit codes

| Code | Meaning |
| ---: | --- |
| 0 | All checks passed |
| 1 | No failures, but at least one warning |
| 2 | At least one check failed |
| 3 | Nav2 Ready itself could not run |

## v0.1 limitations

- ROS 2 Jazzy is the initial target.
- Exactly one obstacle sensor topic is checked per run.
- PointCloud2 validation is structural only; point-cloud quality and voxel-layer
  configuration are not diagnosed.
- Nav2 parameter tuning, URDF validation, costmap configuration, navigation
  performance, rosbag analysis, and automatic fixes are out of scope.

## Test

```bash
colcon test --packages-select nav2_ready
colcon test-result --verbose
```

## License

Apache-2.0

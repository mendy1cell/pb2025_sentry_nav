# Gazebo Sim / Harmonic rotating Livox simulator

This package preserves the data-generation contract of the Classic
`ros2_livox_simulation` package without modifying that reference package.

## Runtime design

The system creates `gz::sim::components::RaycastData` and `WorldPose` on its
custom sensor entity. `PreUpdate` submits the next CSV-ordered batch in the
entity frame only at the 10 Hz scan cadence, then clears it between scans.
Gazebo's Physics system transforms those rays by `WorldPose` and fills the
result vector during `Update`; `PostUpdate` consumes that same ordered result
vector and publishes the ROS messages.

The included model uses the canonical Classic CSV through
`package://ros2_livox_simulation/scan_mode/mid360.csv`. Package URIs are resolved
through the ament index. Do not replace the custom sensor with a fixed-pattern
lidar.

## Preserved scan logic

`scan_pattern.hpp` reads `time,azimuth,zenith`; applies `azimuth * pi/180` and
`zenith * pi/180 - pi/2`; produces `k=current_start_index..+samples` in
`downsample` increments; uses `k % csv_size`; and creates `offset_time` as
`ray_index * (100000000 / ray_count)` nanoseconds. `AxisFromRotate` implements
Classic `Euler(0, zenith, azimuth) * +X`. Ray endpoints are expressed in the
sensor entity frame for the Physics system, while published points remain
`range * axis` in local `livox_frame` coordinates.

The independent regression source in `test/test_scan_pattern.cpp` has no
Gazebo or ROS dependency.

## ROS contract

The plugin publishes at 10 Hz with SensorDataQoS (best-effort, volatile):

| topic | ROS type | frame |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | `livox_frame` |
| `/livox/lidar/pointcloud` | `sensor_msgs/msg/PointCloud2` (`xyz`,`rgb`) | `livox_frame` |

`CustomMsg.header.stamp` and `timebase` come from Gazebo simulation time.
`point_num`, local `x/y/z`, `offset_time`, `tag=0`, and `line=0` follow the
Classic publication loop. Ranges at or outside the configured endpoints are
zeroed. PointCloud2 is auxiliary only and never replaces CustomMsg.

Gazebo Sim 8 `RaycastResultInfo` exposes hit point, hit fraction, and normal,
but no Classic `laser_retro` value or hit entity. The workspace's Classic
worlds use default or explicit zero retro, so `reflectivity=0` preserves their
current behavior without synthesizing intensity. Nonzero Classic `laser_retro`
cannot be represented through this public gz-sim8 component API.

## Robot command and IMU path

`models/sentry_robot/model.sdf` is the native Gazebo Sim robot model. Its
`HolonomicPoseSystem` consumes Gazebo `gz.msgs.Twist` on
`/cmd_vel_chassis`, integrates planar body-frame velocity, and updates only the
model pose. It does not publish odometry or TF; Super-LIO remains the owner of
`/Odometry` and the `odom` to `base` transform.

The native IMU sensor publishes Gazebo IMU messages on `/livox/imu` with
`imu_link` as its frame. `gz_simulation.launch.py` bridges that endpoint to ROS
and bridges ROS Twist commands in the opposite direction. The model keeps the
measured `base_link` to `livox_frame` transform at `0.12 0 0.175`, with zero
rotation.

## Launch modes

The default keeps the standalone sensor world:

```bash
ros2 launch gz_livox_simulation gz_simulation.launch.py
```

For navigation, select an arena alias. `RMUC`, `rmuc_2024`, and `rmuc_2025`
load `RMUC2024.sdf`; `RMUL`, `rmul_2024`, and `rmul_2025` load
`RMUL2024.sdf`; `rmuc_2026` loads `RMUC2026.sdf`. Arena modes create
`models/sentry_robot/model.sdf` at the preserved alias-specific pose.

```bash
ros2 launch gz_livox_simulation gz_simulation.launch.py \
  world:=rmuc_2026 headless:=true
```

An explicit Gazebo Sim SDF path is also accepted as `world`; that mode assumes
the SDF owns its model includes and does not create another sentry.

## Version boundary

Designed for Ubuntu 24.04, ROS 2 Jazzy, `gz-sim8` / Gazebo Harmonic,
`ros_gz_sim`, `ros_gz_bridge`, and `livox_ros_driver2`. The launch uses `gz sim`
and `ros_gz_bridge`, never Gazebo Classic processes.

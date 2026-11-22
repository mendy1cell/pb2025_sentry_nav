# livox_laser_simulation for ROS 2

This is a Gazebo **Classic** ROS 2 port of the original project:
https://github.com/Livox-SDK/livox_laser_simulation. It publishes both
`livox_ros_driver2/msg/CustomMsg` on `<topic>` and `sensor_msgs/msg/PointCloud2`
on `<topic>/pointcloud`.

The current navigation integration targets Ubuntu 24.04 + ROS 2 Jazzy, but the
Gazebo Classic availability and this plugin must be compiled and smoke-tested on
the target machine. It is not an Ignition/Gazebo Sim plugin.

### Usage 
1. clone this repo in your ros2 workspace
```
git clone https://github.com/stm32f303ret6/livox_laser_simulation_RO2.git
```
2. Resolve ROS/system dependencies and build your workspace. Do not rely on a
   second build to hide errors.
```
rosdep install --from-paths src --ignore-src --rosdistro jazzy -r -y
sudo apt install libgoogle-glog-dev libtbb-dev libgflags-dev libpcl-dev libeigen3-dev
colcon build --symlink-install
source install/setup.bash
```
3. include the lidar sensor in your URDF file, for example:
```
<xacro:include filename="$(find ros2_livox_simulation)/urdf/mid70.xacro" />
```
4. attach the sensor to your robot in the URDF (or xacro) file, for example:
```
  <xacro:mid70 name="livox" parent="base_link" topic="mid70">
    <origin xyz="1 0 1" rpy="0 0 0"/>
  </xacro:mid70>
```
you need to specify the parent link (usually base_link)

that's it. the example that i gave you is for mid70, but you can use mid40, mid70, mid360 and so on.

## Super-LIO integration

For the current Mid-360 simulation, the xacro topic is `/livox/lidar`:

- `/livox/lidar`: `livox_ros_driver2/msg/CustomMsg`, SensorData QoS, for
  Super-LIO (`lio.sensor.lidar_type: 1`). Each point has a deterministic
  simulation-scan-relative `offset_time` in nanoseconds.
- `/livox/lidar/pointcloud`: `sensor_msgs/msg/PointCloud2`, SensorData QoS,
  for the existing segmentation/Terrain/Nav2 PointCloud2 consumers.

Do not remap the PointCloud2 endpoint into Super-LIO's Livox mode. The Gazebo
IMU is independently provided by `pb_rm_simulation` on `/livox/imu` with frame
`imu_link`.

thanks to the original repo, you can find more info in it.


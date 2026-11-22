# pb2025_sentry_nav

深圳北理莫斯科大学北极熊战队 2025 赛季哨兵导航项目，支持 Ubuntu 24.04、ROS 2 Jazzy 和 Gazebo Sim 仿真。

本仓库已经包含导航代码、Point-LIO、Livox 仿真插件、Gazebo 世界和机器人模型。下面的步骤从已经打开的仓库目录开始，不需要进入其他工程。

## 1. 系统要求

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Sim 8
- 支持 OpenGL 的显卡驱动

ROS Jazzy 安装文档见 https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html。

每个新终端先执行：

\`\`\`bash
cd ~/VSG/reference/pb2025_sentry_nav

unset AMENT_PREFIX_PATH
unset COLCON_PREFIX_PATH
unset LD_LIBRARY_PATH

source /opt/ros/jazzy/setup.bash
\`\`\`

清理环境是为了避免加载其他工作空间中的旧版 Nav2、BehaviorTree 或 Gazebo 库。

## 2. 安装依赖

在仓库根目录执行：

\`\`\`bash
rosdep install -r \
  --from-paths . \
  --ignore-src \
  --rosdistro jazzy \
  -y
\`\`\`

如果看到 \`pkg_resources is deprecated\`，这是 Python 警告，不是安装失败。

## 3. 编译

第一次编译或更换 ROS 版本后执行：

\`\`\`bash
colcon build \
  --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
\`\`\`

编译成功时最后会看到：

\`\`\`text
Summary: packages finished
\`\`\`

加载当前工作区：

\`\`\`bash
source install/setup.bash
\`\`\`

检查关键包：

\`\`\`bash
ros2 pkg prefix pb2025_nav_bringup
ros2 pkg prefix point_lio
ros2 pkg prefix gz_livox_simulation
\`\`\`

返回路径应位于当前仓库的 \`install/\` 目录。

## 4. 一条命令启动仿真

当前导航启动文件会自动启动 Gazebo、机器人模型、Livox 传感器、点云转换、Point-LIO、Nav2 和 RViz。

推荐调试模式：

\`\`\`bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  slam:=True \
  headless:=False \
  use_composition:=False
\`\`\`

参数含义：

| 参数 | 含义 |
| --- | --- |
| \`world\` | 仿真场地，默认使用 \`rmuc_2025\` |
| \`slam\` | \`True\` 为建图模式，不依赖先验 PCD |
| \`headless\` | \`False\` 显示 Gazebo GUI，\`True\` 只运行服务器 |
| \`use_composition\` | 建议设为 \`False\`，便于查看各节点日志 |

显卡或 Gazebo GUI 出现 EGL/OpenGL 错误时，先使用：

\`\`\`bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  slam:=True \
  headless:=True \
  use_composition:=False
\`\`\`

无 GUI 模式可以验证完整的传感器和导航数据链。Wayland 会话下 Gazebo GUI 崩溃时，登录 Ubuntu on Xorg 后再尝试 GUI 模式。

## 5. 启动后的检查

### Gazebo 模型

看到下面的日志表示机器人已经生成：

\`\`\`text
Entity creation successful.
\`\`\`

### 原始点云

\`\`\`bash
ros2 topic hz /red_standard_robot1/velodyne_points
\`\`\`

正常情况下应能看到约 1.6 Hz 的输出。

### Point-LIO 点云

\`\`\`bash
ros2 topic hz /red_standard_robot1/cloud_registered
\`\`\`

建图模式下，Point-LIO 完成 IMU 初始化后会发布该话题。

### Nav2 动作服务器

\`\`\`bash
ros2 action list
\`\`\`

正常情况下至少包含：

\`\`\`text
/red_standard_robot1/navigate_to_pose
/red_standard_robot1/navigate_through_poses
\`\`\`

如果没有这两个动作，点击 RViz 目标点不会产生运动指令，应查看终端中的 \`bt_navigator\` 日志。

### 速度指令

点击目标点后检查：

\`\`\`bash
ros2 topic hz /red_standard_robot1/cmd_vel_nav2_result
ros2 topic hz /red_standard_robot1/cmd_vel
\`\`\`

Gazebo 底盘最终接收：

\`\`\`text
/cmd_vel_chassis
\`\`\`

## 6. 在 RViz 中导航

1. 等待 RViz 启动。
2. 确认左侧有点云显示。
3. 点击工具栏的 \`Nav2 Goal\`。
4. 在场地中点击目标位置。
5. 拖动鼠标设置目标朝向并释放。
6. 查看终端中的 planner、controller 和 velocity smoother 日志。

默认点云话题：

\`\`\`text
/red_standard_robot1/velodyne_points
\`\`\`

如果 RViz 没有点云，先确认 \`ros2 topic hz\` 有数据，再检查 RViz 的 Fixed Frame 是否为 \`lidar_link\`。

## 7. 建图和保存地图

启动建图：

\`\`\`bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  slam:=True \
  headless:=False \
  use_composition:=False
\`\`\`

保存栅格地图：

\`\`\`bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/rmuc_2025_map \
  --ros-args -r __ns:=/red_standard_robot1
\`\`\`

Point-LIO 生成的点云地图保存在：

\`\`\`text
point_lio/PCD/
\`\`\`

## 8. 先验地图导航

先验 PCD 文件没有放入 Git。将对应文件放到：

\`\`\`text
pb2025_nav_bringup/pcd/simulation/
\`\`\`

然后启动：

\`\`\`bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 \
  slam:=False \
  headless:=False \
  use_composition:=False
\`\`\`

如果提示找不到 \`rmuc_2025.pcd\`，请先使用 \`slam:=True\`，或补充对应的先验文件。

## 9. 常见问题

### rosdep 报 given path 'src' does not exist

当前仓库根目录没有 \`src\` 子目录，使用：

\`\`\`bash
rosdep install -r --from-paths . --ignore-src --rosdistro jazzy -y
\`\`\`

### 找不到 point_lio

\`\`\`bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 pkg prefix point_lio
\`\`\`

如果仍然找不到，重新编译。

### 点击目标点后机器人不动

依次检查：

\`\`\`bash
ros2 action list
ros2 topic hz /red_standard_robot1/cmd_vel_nav2_result
ros2 topic hz /red_standard_robot1/cmd_vel
\`\`\`

如果没有动作服务器，查看 \`bt_navigator\` 日志。如果有动作服务器但没有速度，查看 Point-LIO、TF 和 controller_server 日志。

### 出现 jump back in time 或 imu loop back

说明同时运行了多组 Gazebo、桥接或导航进程。关闭旧进程，只保留一次仿真启动命令，然后重新启动。

### Gazebo 只显示白色方块

当前仿真使用简化机器人模型，文件位于：

\`\`\`text
gz_livox_simulation/models/sentry_robot/model.sdf
\`\`\`

它用于验证传感器、导航和运动控制，不是完整的机器人外观模型。

### Gazebo GUI 出现 EGL/OpenGL 崩溃

先验证无 GUI 模式：

\`\`\`bash
ros2 launch pb2025_nav_bringup rm_navigation_simulation_launch.py \
  world:=rmuc_2025 headless:=True use_composition:=False
\`\`\`

如果无 GUI 模式正常，问题属于本机显卡驱动或 Wayland/OpenGL 会话。登录 Ubuntu on Xorg 后重试。

## 10. 实车启动

实车模式需要实际机器人上的 Livox 驱动、Point-LIO、TF 和底盘接口。

建图模式：

\`\`\`bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  slam:=True \
  use_robot_state_pub:=True
\`\`\`

导航模式：

\`\`\`bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=<YOUR_WORLD_NAME> \
  slam:=False \
  use_robot_state_pub:=True
\`\`\`

## 11. TF 和命名空间

默认命名空间：

\`\`\`text
/red_standard_robot1
\`\`\`

查看 TF：

\`\`\`bash
ros2 run rqt_tf_tree rqt_tf_tree \
  --ros-args \
  -r /tf:=tf \
  -r /tf_static:=tf_static \
  -r __ns:=/red_standard_robot1
\`\`\`

主要坐标系：

\`\`\`text
map
odom
base_link
base_footprint
gimbal_yaw
gimbal_yaw_fake
lidar_link
imu_link
\`\`\`

## 12. 项目结构

\`\`\`text
gz_livox_simulation              Gazebo Sim 传感器和机器人插件
pb_rm_simulation                 仿真场地、模型和世界文件
livox_laser_simulation_RO2       Livox Gazebo 传感器支持
pb2025_nav_bringup               导航与仿真启动文件
point_lio                       Point-LIO 里程计
small_gicp_relocalization        先验地图重定位
ign_sim_pointcloud_tool          仿真点云转换
loam_interface                   点云和里程计坐标变换
sensor_scan_generation           传感器坐标变换和扫描生成
terrain_analysis                 局部地形分析
terrain_analysis_ext             扩展范围地形分析
pb_nav2_plugins                 Nav2 插件
pb_omni_pid_pursuit_controller   全向路径跟踪控制器
fake_vel_transform               速度坐标系转换
\`\`\`

项目使用 Apache-2.0 许可证。

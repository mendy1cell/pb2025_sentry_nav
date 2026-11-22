#!/usr/bin/env python3

import os
from dataclasses import dataclass
from typing import Final

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EqualsSubstitution, LaunchConfiguration
from launch_ros.actions import Node


@dataclass(frozen=True, slots=True)
class WorldConfig:
    world_file: str
    x: str
    y: str
    z: str
    yaw: str


class WorldType:
    RMUC: Final[str] = "RMUC"
    RMUC_2024: Final[str] = "rmuc_2024"
    RMUL: Final[str] = "RMUL"
    RMUL_2024: Final[str] = "rmul_2024"
    RMUC_2025: Final[str] = "rmuc_2025"
    RMUL_2025: Final[str] = "rmul_2025"
    RMUC_2026: Final[str] = "rmuc_2026"


RMUC_2024_CONFIG: Final[WorldConfig] = WorldConfig(
    "RMUC2024.sdf", "6.35", "7.6", "0.2", "0.0"
)
RMUL_2024_CONFIG: Final[WorldConfig] = WorldConfig(
    "RMUL2024.sdf", "4.3", "3.35", "1.16", "0.0"
)
RMUC_2026_CONFIG: Final[WorldConfig] = WorldConfig(
    "RMUC2026.sdf", "6.35", "7.6", "0.2", "0.0"
)
WORLD_CONFIGS: Final[dict[str, WorldConfig]] = {
    WorldType.RMUC: RMUC_2024_CONFIG,
    WorldType.RMUC_2024: RMUC_2024_CONFIG,
    WorldType.RMUC_2025: RMUC_2024_CONFIG,
    WorldType.RMUL: RMUL_2024_CONFIG,
    WorldType.RMUL_2024: RMUL_2024_CONFIG,
    WorldType.RMUL_2025: RMUL_2024_CONFIG,
    WorldType.RMUC_2026: RMUC_2026_CONFIG,
}


def get_world_config(world_type: str) -> WorldConfig | None:
    return WORLD_CONFIGS.get(world_type)


def generate_launch_description() -> LaunchDescription:
    share = get_package_share_directory("pb_rm_simulation")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    robot_description = LaunchConfiguration("robot_description")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    default_robot_description = Command(
        ["xacro ", os.path.join(share, "urdf", "simulation_waking_robot.xacro")]
    )

    def create_world_group(alias: str, config: WorldConfig) -> GroupAction:
        world_path = os.path.join(share, "worlds", config.world_file)
        return GroupAction(
            condition=IfCondition(
                EqualsSubstitution(LaunchConfiguration("world"), alias)
            ),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
                    ),
                    launch_arguments={"gz_args": ["-r ", world_path]}.items(),
                ),
                TimerAction(
                    period=3.0,
                    actions=[
                        Node(
                            package="ros_gz_sim",
                            executable="create",
                            arguments=[
                                "-world",
                                "default",
                                "-name",
                                "robot",
                                "-topic",
                                "robot_description",
                                "-x",
                                config.x,
                                "-y",
                                config.y,
                                "-z",
                                config.z,
                                "-Y",
                                config.yaw,
                            ],
                            output="screen",
                        )
                    ],
                ),
            ],
        )

    return LaunchDescription(
        [
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(share, "meshes")),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use the simulation clock",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=WorldType.RMUC,
                choices=list(WORLD_CONFIGS),
                description="Static RoboMaster arena alias",
            ),
            DeclareLaunchArgument(
                "rviz_config_file",
                default_value=os.path.join(share, "rviz", "rviz2.rviz"),
                description="RViz configuration file",
            ),
            DeclareLaunchArgument(
                "robot_description",
                default_value=default_robot_description,
                description="Robot description",
            ),
            DeclareLaunchArgument("rviz", default_value="false"),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="joint_state_publisher",
                parameters=[
                    {"use_sim_time": use_sim_time, "robot_description": robot_description}
                ],
                output="screen",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                parameters=[
                    {"use_sim_time": use_sim_time, "robot_description": robot_description}
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="gz_clock_imu_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/livox/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(use_rviz),
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_file],
            ),
            *(create_world_group(alias, config) for alias, config in WORLD_CONFIGS.items()),
        ]
    )

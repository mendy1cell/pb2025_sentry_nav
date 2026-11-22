"""Launch the native Gazebo Sim Livox sentry in a selected arena."""

import os
import shlex
from dataclasses import dataclass
from typing import Final

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)

from launch import LaunchContext, LaunchDescription
from launch.action import Action
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.actions.append_environment_variable import (
    AppendEnvironmentVariable,
)
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


@dataclass(frozen=True, slots=True)
class WorldConfig:
    """Installed arena world and legacy sentry spawn pose."""

    world_file: str
    x: str
    y: str
    z: str
    yaw: str


RMUC_2024_CONFIG: Final[WorldConfig] = WorldConfig(
    'RMUC2024.sdf', '6.35', '7.6', '0.2', '0.0'
)
RMUL_2024_CONFIG: Final[WorldConfig] = WorldConfig(
    'RMUL2024.sdf', '4.3', '3.35', '1.16', '0.0'
)
RMUC_2026_CONFIG: Final[WorldConfig] = WorldConfig(
    'RMUC2026.sdf', '6.35', '7.6', '0.2', '0.0'
)
RMUC_2027_CONFIG: Final[WorldConfig] = WorldConfig(
    'rmuc_2027.sdf', '6.35', '7.6', '0.2', '0.0'
)
# Most open flat floor spot of the RMUL 2027 arena (floor top z = 0.10,
# >= 1.4 m clear of walls and steps); z 0.10 drops the chassis ~4 cm.
RMUL_2027_CONFIG: Final[WorldConfig] = WorldConfig(
    'rmul_2027.sdf', '6.5', '6.6', '0.10', '0.0'
)
WORLD_CONFIGS: Final[dict[str, WorldConfig]] = {
    'RMUC': RMUC_2024_CONFIG,
    'rmuc_2024': RMUC_2024_CONFIG,
    'rmuc_2025': RMUC_2024_CONFIG,
    'RMUL': RMUL_2027_CONFIG,
    'rmul_2024': RMUL_2024_CONFIG,
    'rmul_2025': RMUL_2024_CONFIG,
    'rmuc_2026': RMUC_2026_CONFIG,
    'rmuc_2027': RMUC_2027_CONFIG,
    'rmul_2027': RMUL_2027_CONFIG,
}
STANDALONE_WORLD: Final[str] = 'empty_livox'
PHYSICS_ENGINE_ARGUMENT: Final[str] = (
    '--physics-engine gz-physics-dartsim-plugin'
)


def generate_launch_description() -> LaunchDescription:
    """Create the arena, sentry, environment, and bridge launch actions."""
    use_sim_time = LaunchConfiguration('use_sim_time')
    simulation_share = get_package_share_directory('gz_livox_simulation')
    arena_share = get_package_share_directory('pb_rm_simulation')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')
    plugin_path = os.path.join(
        get_package_prefix('gz_livox_simulation'), 'lib'
    )
    model_path = os.path.join(simulation_share, 'models')
    arena_model_path = os.path.join(arena_share, 'meshes')
    empty_world_path = os.path.join(
        simulation_share, 'worlds', 'empty_livox.sdf'
    )
    robot_path = os.path.join(
        simulation_share, 'models', 'sentry_robot', 'model.sdf'
    )
    ld_library_path = ':'.join(
        entry for entry in os.environ.get('LD_LIBRARY_PATH', '').split(':')
        if entry not in ('/opt/MVS/lib/64', '/opt/MVS/lib/32')
    )

    def launch_simulation(context: LaunchContext) -> list[Action]:
        selected_world = LaunchConfiguration('world').perform(context)
        world_config = WORLD_CONFIGS.get(selected_world)

        # 2027 worlds are in gz_livox_simulation/worlds; older ones in pb_rm_simulation/worlds
        if world_config is not None:
            if world_config.world_file in ('rmuc_2027.sdf', 'rmul_2027.sdf'):
                world_path = os.path.join(simulation_share, 'worlds', world_config.world_file)
            else:
                world_path = os.path.join(arena_share, 'worlds', world_config.world_file)
        elif selected_world == STANDALONE_WORLD:
            world_path = empty_world_path
        else:
            world_path = selected_world
        headless_argument = (
            ' -s --headless-rendering'
            if IfCondition(LaunchConfiguration('headless')).evaluate(context)
            else ''
        )
        gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': (
                    '-r'
                    f'{headless_argument}'
                    f' {PHYSICS_ENGINE_ARGUMENT} '
                    f'{shlex.quote(world_path)}'
                )
            }.items(),
        )
        if world_config is None:
            return [gazebo]

        spawn_robot = TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='ros_gz_sim',
                    executable='create',
                    arguments=[
                        '-world',
                        'default',
                        '-name',
                        'sentry_robot',
                        '-file',
                        robot_path,
                        '-x',
                        world_config.x,
                        '-y',
                        world_config.y,
                        '-z',
                        world_config.z,
                        '-Y',
                        world_config.yaw,
                    ],
                    output='screen',
                )
            ],
        )
        return [gazebo, spawn_robot]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'use_sim_time',
                default_value='true',
                description='Use the Gazebo simulation clock',
            ),
            DeclareLaunchArgument(
                'world',
                default_value=STANDALONE_WORLD,
                description=(
                    'Arena alias, empty_livox, or an explicit Gazebo Sim '
                    'SDF path'
                ),
            ),
            DeclareLaunchArgument(
                'headless',
                default_value='false',
                description=(
                    'Run only the Gazebo server with headless rendering'
                ),
            ),
            AppendEnvironmentVariable(
                'GZ_SIM_SYSTEM_PLUGIN_PATH', plugin_path
            ),
            AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', model_path),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', arena_model_path
            ),
            SetEnvironmentVariable('LD_LIBRARY_PATH', ld_library_path),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='gz_navigation_bridge',
                arguments=[
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    '/livox/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                    '/cmd_vel_chassis@geometry_msgs/msg/Twist]gz.msgs.Twist',
                ],
                output='screen',
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_imu_link',
                arguments=[
                    '--x',
                    '-0.011',
                    '--y',
                    '-0.02329',
                    '--z',
                    '0.70412',
                    '--roll',
                    '0.0',
                    '--pitch',
                    '0.0',
                    '--yaw',
                    '0.0',
                    '--frame-id',
                    'base_link',
                    '--child-frame-id',
                    'imu_link',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name='base_link_to_lidar_link',
                arguments=[
                    '--x',
                    '0.0',
                    '--y',
                    '0.0',
                    '--z',
                    '0.66',
                    '--roll',
                    '0.0',
                    '--pitch',
                    '0.0',
                    '--yaw',
                    '0.0',
                    '--frame-id',
                    'base_link',
                    '--child-frame-id',
                    'lidar_link',
                ],
                parameters=[{'use_sim_time': use_sim_time}],
            ),
            OpaqueFunction(function=launch_simulation),
        ]
    )

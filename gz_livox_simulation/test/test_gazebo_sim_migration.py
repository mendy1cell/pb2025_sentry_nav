"""Gazebo Sim migration contract, checked against the current workspace layout.

The simulation entry is navigation/hero2025_nav_bringup/launch/sim_launch.py;
the Gazebo assets live in common/gz_livox_simulation and common/pb_rm_simulation.
"""
import ast
from pathlib import Path

import yaml


SIMULATION_PACKAGE = Path(__file__).resolve().parents[1]
SOURCE_ROOT = SIMULATION_PACKAGE.parents[1]

SIM_LAUNCH = "navigation/hero2025_nav_bringup/launch/sim_launch.py"
GZ_LAUNCH = "common/gz_livox_simulation/launch/gz_simulation.launch.py"
LIVOX_SYSTEM = "common/gz_livox_simulation/src/livox_raycast_system.cpp"
ROBOT_MODEL = "common/gz_livox_simulation/models/sentry_robot/model.sdf"
ARENA_WORLD = "common/pb_rm_simulation/worlds/RMUC2026.sdf"
SUPER_LIO_LAUNCH = "navigation/super_lio/super_lio/launch/hero_super_lio.launch.py"
NAV2_SIM_PARAMS = "navigation/hero2025_nav_bringup/config/nav2_params_sim.yaml"


def source_text(relative_path: str) -> str:
    return (SOURCE_ROOT / relative_path).read_text(encoding="utf-8")


def declared_launch_arguments(relative_path: str) -> set[str]:
    tree = ast.parse(source_text(relative_path))
    return {
        node.args[0].value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "DeclareLaunchArgument"
        and node.args
        and isinstance(node.args[0], ast.Constant)
        and isinstance(node.args[0].value, str)
    }


def test_sim_launch_preserves_public_arguments() -> None:
    assert {
        "use_sim_time",
        "world",
        "mode",
        "backend",
        "headless",
        "relocalization",
        "prior_map",
    } <= declared_launch_arguments(SIM_LAUNCH)


def test_active_simulation_sources_exclude_gazebo_classic() -> None:
    classic_markers = (
        "gazebo_ros",
        "gzserver",
        "gzclient",
        "spawn_entity.py",
        "libgazebo_",
    )
    references = {
        path: tuple(marker for marker in classic_markers if marker in source_text(path))
        for path in (SIM_LAUNCH, GZ_LAUNCH, ROBOT_MODEL, ARENA_WORLD)
        if any(marker in source_text(path) for marker in classic_markers)
    }
    assert not references, f"Gazebo Classic references remain: {references}"


def test_simulation_contract_preserves_topics_types_and_frames() -> None:
    sim_source = source_text(SIM_LAUNCH)
    gz_source = source_text(GZ_LAUNCH)
    livox_source = source_text(LIVOX_SYSTEM)
    lio_source = source_text(SUPER_LIO_LAUNCH)

    # Gazebo bridge and the native Livox CustomMsg raycast.
    assert "ros_gz_bridge" in gz_source
    assert "parameter_bridge" in gz_source
    assert "/livox/imu@sensor_msgs/msg/Imu[gz.msgs.IMU" in gz_source
    assert "livox_ros_driver2::msg::CustomMsg" in livox_source
    assert "livox_frame" in livox_source
    # LIO consumes the sim sensors and publishes the canonical odometry.
    assert "'/livox/lidar'" in sim_source
    assert "'/livox/imu'" in sim_source
    assert "('/lio/odom', '/localization/odometry')" in lio_source
    # Nav2 output reaches the gz chassis input.
    assert "('/control/nav_cmd_vel', '/cmd_vel_chassis')" in sim_source


def test_simulation_gz_uses_dartsim_engine() -> None:
    """Gazebo launch must use DART, not Bullet Featherstone or ODE."""
    launch_source = source_text(GZ_LAUNCH)
    assert "gz-physics-dartsim-plugin" in launch_source
    assert "bullet-featherstone" not in launch_source
    assert "gz-physics-bullet" not in launch_source


def test_simulation_nav2_planner_uses_jazzy_plugin_type() -> None:
    planner = yaml.safe_load(source_text(NAV2_SIM_PARAMS))["planner_server"]["ros__parameters"]
    plugin = planner["GridBased"]["plugin"]
    # pluginlib on Jazzy takes the C++ namespaced class name, not "pkg/Class".
    assert "::" in plugin and "/" not in plugin, plugin


def test_simulation_nav2_bt_navigator_lists_only_custom_plugins() -> None:
    """Jazzy auto-registers its stock BT nodes; listing any again aborts bt_navigator."""
    bt_params = yaml.safe_load(source_text(NAV2_SIM_PARAMS))["bt_navigator"]["ros__parameters"]
    libraries = bt_params.get("plugin_lib_names", [])
    stock = [name for name in libraries if name.startswith("nav2_")]
    assert not stock, f"stock nav2 BT plugins must not be listed: {stock}"


def test_simulation_nav2_uses_jazzy_behavior_plugin_types() -> None:
    nav2_source = source_text(NAV2_SIM_PARAMS)
    for jazzy_type in (
        'plugin: "nav2_behaviors::Spin"',
        'plugin: "nav2_behaviors::DriveOnHeading"',
        'plugin: "nav2_behaviors::Wait"',
    ):
        assert jazzy_type in nav2_source, f"Missing Jazzy type: {jazzy_type}"
    for obsolete_type in (
        'plugin: "nav2_behaviors/Spin"',
        'plugin: "nav2_behaviors/DriveOnHeading"',
        'plugin: "nav2_behaviors/Wait"',
    ):
        assert obsolete_type not in nav2_source, f"Obsolete slash type: {obsolete_type}"
    assert "DriveOnHeading<>" not in nav2_source
    # The custom BackUpFreeSpace behavior is preserved.
    assert 'plugin: "pb_nav2_behaviors/BackUpFreeSpace"' in nav2_source


def test_livox_raycast_uses_world_pose_on_dedicated_query_entity() -> None:
    livox_source = source_text(LIVOX_SYSTEM)
    compact_source = " ".join(livox_source.split())
    query_pose = (
        "ecm.CreateComponent( raycast_entity_, "
        "gz::sim::components::Pose(gz::sim::worldPose(entity_, ecm)))"
    )
    query_raycast = (
        "ecm.CreateComponent(raycast_entity_, "
        "gz::sim::components::RaycastData())"
    )

    # Physics sees Pose and RaycastData together without changing the sensor pose.
    assert "raycast_entity_ = ecm.CreateEntity();" in compact_source
    assert query_pose in compact_source
    assert query_raycast in compact_source
    assert "pose->Data() = gz::sim::worldPose(entity_, ecm);" in compact_source
    assert compact_source.count(
        "Component<gz::sim::components::RaycastData>(raycast_entity_)"
    ) == 2
    assert "components::RaycastData>(entity_)" not in compact_source
    assert "components::Pose>(entity_)" not in livox_source

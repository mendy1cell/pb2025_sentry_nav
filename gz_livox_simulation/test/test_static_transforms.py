import ast
from pathlib import Path


LAUNCH_PATH = (
    Path(__file__).resolve().parents[1] / "launch" / "gz_simulation.launch.py"
)
EXPECTED_TRANSFORMS = {
    (
        "base_link",
        "imu_link",
        "-0.011",
        "-0.02329",
        "0.70412",
    ),
    (
        "base_link",
        "lidar_link",
        "0.0",
        "0.0",
        "0.66",
    ),
}


def test_gazebo_sim_publishes_measured_robot_static_transforms() -> None:
    # Given: the shared native Gazebo Sim launch used by standalone and full sim.
    launch_tree = ast.parse(LAUNCH_PATH.read_text(encoding="utf-8"))

    # When: static transform publishers and their launch configuration are read.
    publishers = []
    for call in (
        node
        for node in ast.walk(launch_tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == "Node"
    ):
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        if not (
            isinstance(keywords.get("package"), ast.Constant)
            and keywords["package"].value == "tf2_ros"
            and isinstance(keywords.get("executable"), ast.Constant)
            and keywords["executable"].value == "static_transform_publisher"
        ):
            continue
        arguments = ast.literal_eval(keywords["arguments"])
        publishers.append((arguments, keywords.get("parameters")))

    transforms = {
        (
            arguments[arguments.index("--frame-id") + 1],
            arguments[arguments.index("--child-frame-id") + 1],
            arguments[arguments.index("--x") + 1],
            arguments[arguments.index("--y") + 1],
            arguments[arguments.index("--z") + 1],
        )
        for arguments, _ in publishers
    }

    # Then: all measured frames are published once and follow simulation time.
    assert transforms == EXPECTED_TRANSFORMS
    assert len(publishers) == len(EXPECTED_TRANSFORMS)
    for _, parameters in publishers:
        assert ast.unparse(parameters) == "[{'use_sim_time': use_sim_time}]"

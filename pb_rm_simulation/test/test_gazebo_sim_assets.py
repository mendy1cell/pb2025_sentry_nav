import sys
from importlib.util import module_from_spec, spec_from_file_location
from pathlib import Path
from types import ModuleType
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORLD_MODELS = {
    "RMUC2024.sdf": "model://RMUC2024_world",
    "RMUC2026.sdf": "model://RMUC2026_world",
    "RMUL2024.sdf": "model://RMUL2024_world",
}
WORLD_SYSTEMS = {
    "gz::sim::systems::Physics",
    "gz::sim::systems::Sensors",
    "gz::sim::systems::SceneBroadcaster",
}


def load_simulation_launch() -> ModuleType:
    launch_path = PACKAGE_ROOT / "launch" / "rm_simulation.launch.py"
    spec = spec_from_file_location("rm_simulation_launch", launch_path)
    assert spec is not None
    assert spec.loader is not None
    module = module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_native_worlds_use_dart_bullet_collision_detector() -> None:
    """All arena worlds must use DART with Bullet for Livox raycast support."""
    for world_name in WORLD_MODELS:
        root = ElementTree.parse(PACKAGE_ROOT / "worlds" / world_name).getroot()
        physics = root.find("world/physics")
        assert physics is not None, f"{world_name}: missing physics element"
        dart = physics.find("dart")
        assert dart is not None, f"{world_name}: missing dart element in physics"
        collision_detector = dart.find("collision_detector")
        assert collision_detector is not None, (
            f"{world_name}: missing collision_detector in dart"
        )
        assert collision_detector.text == "bullet", (
            f"{world_name}: expected bullet collision_detector, got {collision_detector.text!r}"
        )


def test_native_worlds_reference_installed_static_models() -> None:
    for world_name, model_uri in WORLD_MODELS.items():
        root = ElementTree.parse(PACKAGE_ROOT / "worlds" / world_name).getroot()
        world = root.find("world")

        assert root.attrib["version"] >= "1.9"
        assert world is not None
        assert {plugin.attrib["name"] for plugin in world.findall("plugin")} >= WORLD_SYSTEMS
        assert model_uri in {uri.text for uri in world.findall("include/uri")}
        assert not world.findall("model/plugin")


def test_native_arena_models_preserve_collision_and_visual_meshes() -> None:
    for model_name in ("RMUC2024_world", "RMUC2026_world", "RMUL2024_world"):
        model_root = PACKAGE_ROOT / "meshes" / model_name
        root = ElementTree.parse(model_root / "model.sdf").getroot()
        mesh_uris = {uri.text for uri in root.findall(".//mesh/uri")}

        assert root.attrib["version"] >= "1.9"
        assert len(mesh_uris) == 1
        assert next(iter(mesh_uris)).startswith(f"model://{model_name}/meshes/")
        assert (model_root / "model.config").is_file()


def test_world_aliases_preserve_spawn_pose_mapping() -> None:
    launch_module = load_simulation_launch()
    expected = {
        "RMUC": ("RMUC2024.sdf", "6.35", "7.6", "0.2", "0.0"),
        "rmuc_2024": ("RMUC2024.sdf", "6.35", "7.6", "0.2", "0.0"),
        "rmuc_2025": ("RMUC2024.sdf", "6.35", "7.6", "0.2", "0.0"),
        "RMUL": ("RMUL2024.sdf", "4.3", "3.35", "1.16", "0.0"),
        "rmul_2024": ("RMUL2024.sdf", "4.3", "3.35", "1.16", "0.0"),
        "rmul_2025": ("RMUL2024.sdf", "4.3", "3.35", "1.16", "0.0"),
        "rmuc_2026": ("RMUC2026.sdf", "6.35", "7.6", "0.2", "0.0"),
    }

    assert {
        alias: (
            config.world_file,
            config.x,
            config.y,
            config.z,
            config.yaw,
        )
        for alias, config in launch_module.WORLD_CONFIGS.items()
    } == expected

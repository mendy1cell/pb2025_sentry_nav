from pathlib import Path
from xml.etree import ElementTree


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
MODEL_PATH = PACKAGE_ROOT / "models" / "sentry_robot" / "model.sdf"
WORLD_PATH = PACKAGE_ROOT / "worlds" / "empty_livox.sdf"
LAUNCH_PATH = PACKAGE_ROOT / "launch" / "gz_simulation.launch.py"
RMUC_WORLD_PATH = (
    PACKAGE_ROOT.parent / "pb_rm_simulation" / "worlds" / "RMUC2024.sdf"
)


def test_sentry_model_preserves_frames_and_extrinsics() -> None:
    # Given: the native Gazebo Sim robot description.
    root = ElementTree.parse(MODEL_PATH).getroot()
    model = root.find("model")

    # Sensors share one rigid body to avoid fixed-joint acceleration artifacts.
    base = model.find("link[@name='base_link']")
    frames = {f.attrib["name"] for f in model.findall("frame")}

    # Then: navigation and sensor frame contracts match the measured mounting.
    assert {"base_footprint", "imu_link", "lidar_link"} <= frames
    assert base.findtext("pose") == "0 0 0.10 0 0 0"
    assert base.find("sensor[@type='imu']").findtext("pose") == "-0.011 -0.02329 0.70412 0 0 0"
    lidar = base.find("sensor[@type='custom']")
    # Inverted mount (roll pi) with the matching install attitude, so the
    # published cloud is upright in lidar_link like the real driver output.
    assert lidar.findtext("pose") == "0 0 0.66 3.14159265358979 0 0"
    assert lidar.findtext("plugin/install_attitude_rpy") == "3.14159265358979 0 0"


def test_sentry_model_uses_native_gazebo_plugins_without_odom_or_tf() -> None:
    # Given: the native Gazebo Sim robot description.
    root = ElementTree.parse(MODEL_PATH).getroot()
    model = root.find("model")

    # When: plugin and sensor configuration is inspected structurally.
    controller = model.find("plugin[@name='gz_livox_simulation::HolonomicPoseSystem']")
    imu = model.find("link[@name='base_link']/sensor[@type='imu']")
    livox = model.find("link[@name='base_link']/sensor[@type='custom']/plugin")
    model_xml = ElementTree.tostring(model, encoding="unicode")

    # Then: only command, IMU, and Livox contracts are configured.
    assert controller is not None
    assert controller.findtext("topic") == "/cmd_vel_chassis"
    assert imu is not None
    assert imu.findtext("topic") == "/livox/imu"
    assert imu.findtext("gz_frame_id") == "imu_link"
    assert livox is not None
    assert livox.findtext("frame_id") == "lidar_link"
    assert livox.findtext("topic") == "/livox/lidar"
    assert all(marker not in model_xml for marker in ("gazebo_ros", "libgazebo_", "Odometry", "<tf"))


def test_empty_world_spawns_sentry_at_preserved_pose() -> None:
    # Given: the package's executable Gazebo Sim world.
    root = ElementTree.parse(WORLD_PATH).getroot()
    world = root.find("world")

    # When: the sentry model include is selected.
    includes = {
        include.findtext("uri"): include.findtext("pose")
        for include in world.findall("include")
    }

    # Then: the default RMUC spawn pose remains unchanged.
    assert includes["model://sentry_robot"] == "6.35 7.6 0.2 0 0 0"


def test_launch_selects_raycast_capable_physics() -> None:
    # Given: the Gazebo Sim launch description and RMUC world.
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    rmuc_world = ElementTree.parse(RMUC_WORLD_PATH).getroot().find("world")

    # When: Gazebo server arguments and DART collision settings are inspected.
    physics_argument = "--physics-engine gz-physics-dartsim-plugin"
    collision_detector = rmuc_world.findtext(
        "physics/dart/collision_detector"
    )

    # Then: the backend exposes ray intersections through Bullet collision queries.
    assert physics_argument in launch_source
    assert collision_detector == "bullet"


def test_rmuc2026_floor_slab_has_primitive_collision() -> None:
    # Given: the RMUC 2026 arena, whose 0.2 m floor slab top is two huge mesh
    # triangles that a box chassis sinks through under Bullet trimesh contact.
    import struct

    arena = PACKAGE_ROOT.parent / "pb_rm_simulation" / "meshes" / "RMUC2026_world"
    link = ElementTree.parse(arena / "model.sdf").getroot().find("model/link")
    data = (arena / "meshes" / "RMUC2026_world.stl").read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    slab = []
    for i in range(count):
        v = struct.unpack_from("<9f", data, 84 + 50 * i + 12)
        xs, ys, zs = v[0::3], v[1::3], v[2::3]
        if all(abs(z * 0.001 - 0.2) < 1e-6 for z in zs) and (
            max(xs) - min(xs)
        ) * (max(ys) - min(ys)) * 1e-6 > 200.0:
            slab.extend(zip(xs, ys))

    # When: the primitive floor collision is located.
    floor = link.find("collision[@name='floor_slab']")
    pose = [float(x) for x in floor.findtext("pose").split()]
    size = [float(x) for x in floor.findtext("geometry/box/size").split()]

    # Then: it covers the mesh slab exactly, top face at the slab top z=0.2.
    xs = [x * 0.001 for x, _ in slab]
    ys = [y * 0.001 for _, y in slab]
    assert len(slab) == 6
    assert abs(pose[0] - size[0] / 2 - min(xs)) < 1e-3
    assert abs(pose[0] + size[0] / 2 - max(xs)) < 1e-3
    assert abs(pose[1] - size[1] / 2 - min(ys)) < 1e-3
    assert abs(pose[1] + size[1] / 2 - max(ys)) < 1e-3
    assert abs(pose[2] + size[2] / 2 - 0.2) < 1e-6
    assert link.find("collision[@name='collision']/geometry/mesh") is not None


def test_sentry_chassis_contacts_are_spheres_spanning_the_footprint() -> None:
    # Given: the chassis collision set. A box collision sinks ~15 cm into the
    # RMUC arena's slope/step trimesh under Bullet; spheres stay on top.
    base = ElementTree.parse(MODEL_PATH).getroot().find("model/link[@name='base_link']")
    collisions = base.findall("collision")

    # When: each contact's shape and offset is read.
    shapes = {c.find("geometry")[0].tag for c in collisions}
    radii = {float(c.findtext("geometry/sphere/radius")) for c in collisions}
    offsets = [[float(v) for v in c.findtext("pose").split()[:3]] for c in collisions]

    # Then: only spheres, filling the 0.30 x 0.30 x 0.12 envelope with no gaps
    # wider than one radius between neighbours.
    assert shapes == {"sphere"}
    assert radii == {0.06}
    assert len(collisions) == 9
    assert {(x, y) for x, y, _ in offsets} == {
        (x, y) for x in (-0.09, 0.0, 0.09) for y in (-0.09, 0.0, 0.09)
    }
    assert all(z == 0.0 for _, _, z in offsets)

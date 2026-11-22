# Gazebo Livox Simulation Worlds

This directory contains world files for Gazebo simulation with the Livox Mid-360 lidar.

## World Files

### `empty_livox.sdf`
Original baseline world with a flat 100×100m ground plane. No terrain geometry.

### `rmuc_2027.sdf` — RoboMaster University Championship 2027 Arena
Competition arena terrain for RMUC 2027.

**Terrain specs:**
- Mesh: `meshes/rmuc_2027.stl` (18.9 MB, 378,666 triangles)
- Dimensions: 28.0 × 16.0 × 3.8 meters (actual competition arena size)
- Origin offset: bounding box starts at (0.15, 0.06, 0.02)
- Friction coefficient: μ = 0.9 (indoor floor)

**Launch:**
```bash
gz sim rmuc_2027.sdf
```

### `rmul_2027.sdf` — RoboMaster University League 2027 Track
Outdoor track terrain for RMUL 2027.

**Terrain specs:**
- Mesh: `meshes/rmul_2027.stl` (187 KB, 3,811 triangles)
- Raw dimensions: 12,600 × 520 × 8,600 meters (requires 0.001 scale applied)
- Scaled dimensions: 12.6 × 0.52 × 8.6 meters
- Friction coefficient: μ = 0.9

**Note:** RMUL mesh uses millimeter units in the STL. The world file applies a 0.001 scale factor to convert to meters. If the scaled terrain is still too large or positioned incorrectly, adjust the `<scale>` tag in the `<mesh>` element or add a `<pose>` to the model.

**Launch:**
```bash
gz sim rmul_2027.sdf
```

## Mesh Files

Terrain STL files are located in `meshes/` subdirectory:
- `meshes/rmuc_2027.stl` — RMUC 2027 arena geometry
- `meshes/rmul_2027.stl` — RMUL 2027 track geometry

Both meshes use:
- Binary STL format
- Static collision geometry (non-deformable)
- Gray material (ambient 0.5, diffuse 0.7)
- ODE friction parameters (μ = μ₂ = 0.9)

## Robot Spawn Position

All worlds spawn `model://sentry_robot` at pose `(6.35, 7.6, 0.2, 0, 0, 0)` (x, y, z, roll, pitch, yaw). Adjust spawn height or position if the robot clips through or floats above the terrain.

## Validation

SDF syntax validation with `gz sdf -k <world.sdf>` will report an error about `model://sentry_robot` not being found. This is expected when running validation outside the full Gazebo environment with model paths configured. The terrain mesh references (`meshes/*.stl`) use relative paths and will resolve correctly at runtime.

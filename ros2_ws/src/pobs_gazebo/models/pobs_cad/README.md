# POBS CAD visual

This is an optional version of the POBS robot with a CAD chassis visual. The
default `lake.sdf` world still loads the simple `pobs` model. Use
`worlds/lake_cad.sdf` to load this version instead.

The CAD export is intentionally ignored by Git. Place `Chassis.STL` in this
directory's `meshes/` subdirectory before using the CAD world. The STL is
scaled from millimeters and centered by `model.sdf`. The collision and physics
properties remain the same as the simple robot.

From the Ubuntu WSL host use `pobs sim --cad` (optionally `--headless`). After
changing the default robot or world, run
`python3 ~/ros2_ws/scripts/sync_robot_variants.py` inside the container to update
this variant. This retains the four sonar mounts and descriptive motor names.

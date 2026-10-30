# Lake mapping test area

The default lake now loads `models/lake_terrain/meshes/terrain_bumpy.obj` for
both its visual and collision mesh. The original `terrain.obj` is the baseline
and remains unchanged. Bumps are smooth, fade out near the shoreline, and use
seed 42. Three larger mounds and a shallow trench sit near the mapping area.
The water surface stays at z=0.

`models/sonar_targets/model.sdf` contains three irregular boulders and a wall.
Each has matching visual and collision geometry. Their heights are calculated
from the generated terrain; the bases extend slightly into it. Objects are
static, so they remain fixed when the world runs or resets.

The robot starts at (-12, -8, -6.314902), 2.5 m above its local bottom. The
forward wall is centered at (-6.8, -8), 6 m wide and 4.5 m tall. At the initial
level pose, the expected ranges are approximately:

| Sonar | Closest range |
| --- | --- |
| Left | 2.64 m |
| Right | 2.84 m |
| Middle | 4.56 m (wall) |
| Downward | 2.42 m |

The free vehicle will move under physics after startup; these are initial-pose
references, not fixed sensor outputs or a depth-hold controller.

## Regenerate

Inside the simulator container (`pobs shell` from the Ubuntu WSL host):

```bash
python3 ~/ros2_ws/scripts/generate_lake_terrain.py
```

Optional controls:

```bash
python3 ~/ros2_ws/scripts/generate_lake_terrain.py \
  --seed 42 --roughness 0.35 --feature-scale 1.0
```

`roughness` controls the small smooth variations in meters. `feature-scale`
scales the mound and trench heights. The script also regenerates and grounds
the boulders and wall, recalculates mesh normals, and prints a recommended
robot spawn 2.5 m above the bottom. It does not edit the world spawn for you;
update `worlds/lake.sdf` if a changed terrain needs a different starting height.
Repeating the same command produces identical assets without stacking bumps.

## Reload and view

Geometry changes require a full reload. On the Ubuntu WSL host:

```bash
pobs stop
pobs sim
pobs build
pobs sensors source:=simulation
```

In another WSL terminal:

```bash
pobs rviz point_cloud_mapping
```

Under **Lake reference**, terrain, water and **Sonar test objects** have separate
visibility controls. The terrain and objects are reference geometry; the
accumulated sonar map still uses only measured ranges and Gazebo truth. Both
use the fixed blue depth palette. Move the vehicle to sample more surfaces;
four stationary beams do not produce a dense map of the whole lake.

For subsequent runs, `pobs reset` resets the loaded world without closing its
nodes or windows. It does not reload mesh or SDF edits.

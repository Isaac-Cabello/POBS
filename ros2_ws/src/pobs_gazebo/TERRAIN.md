# Natural lake bathymetry

The default lake uses connected landforms sculpted into
`models/lake_terrain/meshes/terrain_bumpy.obj`, with identical visual and
collision geometry. This is synthetic terrain, not a measured real-world lake.
The original `terrain.obj`, shoreline, above-water land and water at z=0 remain
unchanged.

The visual mesh has elevation texture coordinates and a generated color ramp
in `materials/elevation.png`. Deep areas are blue, slopes teal and green, and
the shoreline sandy, matching the terrain preview. Both Gazebo and the RViz
reference surface use `materials/elevation_palette.json` over z=-15 to z=4 m.
Regeneration rebuilds the texture and coordinates; the collision surface is
unchanged. Water is highly transparent so the colors remain visible.

- A winding drowned river channel crosses the lake west to east, with a
  4–7 m characteristic width and about 3.4 m of incision.
- A broad southwestern basin adds about 2.4 m of depth.
- An uneven northern ridge rises about 4–5 m, with a steep southern face and
  a gentler back slope.
- A northern shelf has a curved drop-off and about 3.5 m of relief.
- An eastern sediment fan spreads over roughly 24 by 18 m at the channel mouth.

Features blend into the existing bed and fade smoothly near shore. Small seeded
roughness adds surface variation. There are no freestanding primitive obstacles
in either default lake world. The robot starts at (-12, -8, -9.032295), 2.5 m above
the local bottom; forward returns now depend on the terrain and vehicle pose.

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

`roughness` controls small smooth variations in meters. `feature-scale` scales
large landforms. Both at zero restore the baseline. Repeating a command produces
identical assets without stacking deformations. Normals are recomputed, and
submerged features stay at least 0.8 m below water.

The script prints a recommended robot spawn 2.5 m above the generated bed.
After changing controls, update the robot pose in **both** `worlds/lake.sdf` and
`worlds/lake_cad.sdf` using that value; generation does not edit world files.

## Optional sonar calibration targets

Generation also grounds three boulders and a rectangular wall in
`models/sonar_targets/model.sdf`, with matching visual and collision geometry.
To use these artificial targets, add this include under `<world>` in the desired
lake world, then reload:

```xml
<include>
  <uri>model://sonar_targets</uri>
  <name>sonar_targets</name>
  <pose>0 0 0 0 0 0</pose>
</include>
```

## Reload and view

Geometry changes require a full reload. On the Ubuntu WSL host:

```bash
pobs stop
pobs sim
```

In another WSL terminal, build installed reference assets and start the view:

```bash
pobs build
pobs rviz point_cloud_mapping
```

Under **Lake reference**, enable **Lake terrain** to inspect the complete
surface. Water has a separate visibility control. **Sonar test objects** is
empty unless the optional fixtures are included. Reference surfaces are known
simulation geometry; the sonar map still uses only measured ranges and Gazebo
truth. Measured sonar data retains its blue depth palette, which saturates below
10 m; the reference terrain palette distinguishes depths down to 15 m.

Move or survey the vehicle to sample the features. Four stationary beams do
not produce a dense map of the whole lake. `pobs reset` resets the loaded world
without reloading mesh or SDF edits.

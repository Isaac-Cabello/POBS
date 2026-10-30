# POBS visualization

Start Gazebo and the sensor pipeline first (`pobs sensors source:=simulation`;
append `existing_sensors:=false` if the existing bridge already runs).
Then use a second Ubuntu WSL host terminal (the host launcher handles Docker):

```bash
pobs rviz                               # Both display groups
pobs rviz point_cloud_mapping           # Mapper and mapping displays
pobs rviz state_estimation              # Estimated state display only
pobs rvis point_cloud_mapping           # Alias
pobs rviz point_cloud_mapping rviz:=false  # Mapper without a GUI
```

Both `state_estimation` and `point_cloud_mapping` groups are always present in
RViz and can be toggled in Displays. Tags choose which groups start enabled.
Only an enabled mapping tag starts `pobs_mapping/point_cloud_mapper`.
Enabling a display later in RViz does not start a disabled mapper.

The mapping group displays the entire accumulated `/pobs/mapping/points` cloud,
with no RViz decay, and `/pobs/mapping/ground_truth_pose`. Its fixed frame is
`lake`. Mapping uses Gazebo truth exclusively, independent of estimated state.

Every view also starts a reference scene publisher that registers `lake` in TF
without needing the estimator. The `Lake reference` display shows a translucent
copy of the terrain loaded from the actual world SDF and OBJ geometry. The water
surface is available as a separate display, initially disabled to keep the
underwater points visible. These reference surfaces are simulation geometry;
the `Accumulated sonar map` display contains the measured sonar points.
The **Sonar test objects** display shows the three boulders and wall read from
the world's included `sonar_targets` model, using the same poses and meshes as
Gazebo. It can be toggled independently of the reference terrain.

Sonar hits and terrain use a fixed blue depth scale: pale blue at the water
surface (z=0), dark blue at 10 m depth (z=-10). Values outside this interval
use the nearest endpoint color. Colors represent depth below water, rather
than distance along a sonar beam or sensor identity. The cloud uses RViz's
Intensity transformer with `Channel Name: z` and `Use rainbow: false`; no
intensity measurement is added to the sonar data. Change `Min Intensity`
(negative depth in meters) to extend the displayed range for a deeper world.

The Move Camera tool starts active: drag the left mouse button to orbit,
drag the middle mouse button to pan, and use the scroll wheel to zoom.
The toolbar also provides Interact, Select, Focus Camera, and Measure tools.

State estimation starts the estimator and comparison node, restoring the green
ground-truth and orange estimated paths. `pobs compare` selects this view too.
Pass `start_estimator:=false` if an estimator already runs. The comparison uses
the latest pair of samples within 50 ms and retains up to 2,000 trail poses;
the sonar map still retains every accepted point without this limit.
A display-only static `lake -> odom` transform
aligns estimator horizontal zero with the robot spawn x/y parsed from
`worlds/lake.sdf`; z remains zero because estimated z already uses water level.
This does not improve estimator accuracy or supply input to the mapper.
Override `estimator_offset_x:=... estimator_offset_y:=...` for another origin.
Do not run another conflicting publisher for this transform.
Pass `publish_odom_transform:=false` if another node owns it. Both paths are
published directly in `lake`, so RViz can display them across clock resets.
Soft reset clears the trails and map; reference geometry refreshes every two
wall-clock seconds, even while the simulation is paused.

ROS launch arguments follow tags and override defaults, including
`model_path:=...`, `world_path:=...`, `world_frame:=lake`, `world_name:=lake`,
and `use_sim_time:=true`. The launch bridges the world's clock into `/clock`.
`POBS_WORKSPACE` selects the source workspace (default `~/ros2_ws`).

Build: `colcon build --packages-up-to pobs_visualization` and source
`install/setup.bash`. The `pobs` shell command is installed by the container
setup; source edits are available directly when its script is mounted.

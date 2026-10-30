# POBS behavior retained from earlier development

The October 24 compatibility review compared the current sonar/mapping work
against all seven earlier POBS chats: ROS installation, Turtlesim setup,
software overview, Gazebo movement, RViz comparison, optional CAD, and IMU/depth
integration. Newer requests take precedence over older ones.

| Area | Current intended behavior |
| --- | --- |
| Host commands | Run `pobs` in Ubuntu WSL; it handles Docker. `pobs sim` opens the Gazebo GUI; `--headless` selects a server. `pobs rvis` remains an alias. |
| State visualization | `pobs compare` and `pobs rviz state_estimation` start the estimator and comparison view: green truth trail, orange estimated trail, actual robot pose, lake reference. |
| Comparison | Pair the latest estimate and truth within 50 ms, consume each pair once, retain 2,000 trail poses, and clear both on clock rewind. Numeric position-error calculation remains the earlier TODO. |
| Frames and depth | Estimate x/y start at zero and are translated to the lake spawn for display. Estimated z is negative absolute water depth; no spawn-z offset is applied. Paths are published in `lake` to survive RViz TF resets. |
| Estimator | Restore the committed IMU acceleration integration for x/y, using sensor timestamps, quaternion rotation, gravity compensation, and the existing deadband. Use the separate `/pobs/depth` measurement directly for z. |
| Mapping | Use four sonar ranges and Gazebo truth exclusively. Read mounts from SDF; match/interpolate truth at acquisition time; retain every accepted point for the run. No estimator input, expiry, downsampling or point cap. |
| Sonars | Left/right slope down 45 degrees at the front; middle faces forward; downward sensor sits under the robot. Each publishes one `Range` distance on its own topic. Downward simulation retains the original 0.15–30 m behavior; front sensors use 0.05–6 m. |
| Separate depth sensor | `/pobs/depth` remains positive absolute depth below water, and `/pobs/pressure` remains pressure. The downward sonar reports bottom clearance and does not replace these inputs. |
| RViz appearance | State and mapping retain separate named groups. Sonar cloud and new terrain use the requested fixed blue depth palette. Water display starts disabled. Camera controls start active. |
| Reference geometry | Read the actual bumpy terrain, three boulders and wall from the world/model files. Republish geometry and root TF every two wall-clock seconds so reset/paused simulation keeps its context. |
| Soft restart | `pobs reset [--paused]` keeps the container/nodes/windows, restores Gazebo buoyancy metadata, and clears old paths/map on time rollback. Edited geometry requires a full reload. |
| Motors | Restore `front_left`, `front_right`, `back_left`, `back_right`, `left_thruster`, `right_thruster` and their matching `_joint` names. Preserve their existing poses, physics and force settings. Motor shortcuts remain removed as requested. |
| CAD | Simple chassis remains default. `pobs sim --cad` uses the optional local STL; its physics, sonar mounts, terrain and spawn match the default. The large STL remains ignored by Git. |

The temporary two-wheel rotate/drive practice and its shortcuts were explicitly
discarded when the lake codebase replaced them. They are not restored as AUV
controllers. Native Windows ROS launch scripts were superseded by the team's
Docker/WSL workflow. The onboarding models and documentation remain available.

The stabilizer is still work in progress; this review does not add depth hold,
an autonomous mission state machine, visual odometry, or SLAM. IMU-only horizontal
position can drift. Gazebo GPU sonars can return decorative water geometry when
the vehicle tilts upward; those measured returns are currently retained by the
mapper. Hardware UART behavior still requires the physical sensors to validate.

## Commands

From Ubuntu WSL:

```bash
pobs sim                       # Or: pobs sim --headless
pobs build
pobs sensors source:=simulation
```

Keep sensors running, then use a second WSL terminal:

```bash
pobs rviz state_estimation point_cloud_mapping
# State comparison only:
pobs compare
# Map only:
pobs rviz point_cloud_mapping
```

Use one visualization launch at a time. Selecting groups in RViz changes display
visibility; it does not launch nodes disabled by the original command's tags.
For an external estimator, append `start_estimator:=false`. The host launcher
detects an already running standard estimator executable automatically.

```bash
pobs reset
pobs sonar left --watch
pobs sonar right --watch
pobs sonar middle --watch
pobs sonar depth --watch
```

After changing `scripts/pobs`, install it on the WSL host from the repository
root with `install -Dm755 scripts/pobs ~/.local/bin/pobs`. After changing default
robot physics/mounts or the lake world, regenerate the optional CAD variant inside
the container with `python3 ~/ros2_ws/scripts/sync_robot_variants.py`.

## Verification

Functional regression tests and a live Gazebo/RViz check accompany this review.
The compatibility test runner is `research_sen0599/audit_compatibility.sh`.
It exercises state comparison and resets, IMU motion, absolute depth, four-sonar
drivers/adapters, SDF projection and accumulation, scene geometry, terrain,
CAD/model consistency, and host command routing in an isolated ROS domain.

The ROS workspace build and 88 functional regression checks passed. In a separate
Gazebo partition, 107 measured sonar endpoints agreed with independent terrain /
object mesh intersections (range tolerance 8 cm; endpoint tolerance 12 cm).
The live combined view produced all four sonar streams, IMU, pose, pressure,
depth, odometry, both paths and accumulated points. A live soft reset cleared
both trails and cloud, preserved all six buoyancy metadata entries, and rebuilt
the paths/map while reference geometry continued publishing. The six descriptive
motor command topics were present in Gazebo. RViz showed Global Status OK.

The scan and clock bridges now share a managed process wrapper so shutdown sends
one termination to each bridge. Simulation adapters use bounded waits so they
also stop promptly when sensor data or simulation time is paused.
Optional CAD consistency was checked against the simple model and current world.
A separate live CAD Gazebo run also passed: all four sonar ranges matched the
same terrain/object intersections, its 100 accumulated endpoints landed on the
expected surfaces, and all sensor/mapper/scene processes shut down cleanly.

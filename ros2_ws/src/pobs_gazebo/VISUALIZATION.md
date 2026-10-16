# POBS visualization

View the robot's ground-truth and estimated paths in RViz.

## Run with the local `pobs` commands

Run these commands in **WSL Ubuntu**, with Docker Desktop running and Ubuntu
WSL integration enabled. All processes run in the same `pobs-lake` container.
The local helper at `/home/kotha/.local/bin/pobs` uses
`/mnt/c/Users/kotha/pobs` as its checkout; it is not installed by cloning this repo.

### 1. Start Gazebo and build

In WSL terminal 1:

```bash
pobs sim
pobs build
```

`pobs sim` opens the Gazebo GUI. Use `pobs sim --headless` instead if you only
want the Gazebo server and RViz. Build on first use and after Python package,
launch, or dependency changes; a reset does not require a rebuild.

### 2. Start the sensor and clock bridge

In WSL terminal 2:

```bash
pobs bridge
```

Leave this terminal running. It bridges Gazebo pose, IMU, sonar, and `/clock`,
and starts the depth/pressure publisher used by the estimator.

### 3. Start the estimator, comparison, and RViz

In WSL terminal 3:

```bash
pobs compare
```

Leave this terminal running. This launches the existing state estimator,
the trajectory comparison node, and RViz with the saved configuration.
All three use simulation time. Start only one instance of each node.

Useful commands from another WSL terminal:

```bash
pobs status          # Simulator and bridge status
pobs topics          # ROS topic names
pobs pose            # One ground-truth PoseStamped message
pobs reset           # Reset Gazebo in place, restore buoyancy, and resume
pobs reset --paused  # Same reset, but wait before playing
pobs shell           # Enter a sourced ROS/workspace shell in this container
```

`pobs rviz` opens only the saved RViz view; use it when RViz is not already
running. It does not start the estimator or comparison node. To stop the
foreground launches, press Ctrl+C in their terminals. `pobs stop` stops and
removes the simulator container; starting it again requires restarting the
bridge and comparison launch.

## Run with standard container and ROS shell commands

This is an alternative to the helper workflow above. Do not launch a second
simulator, bridge, estimator, or comparison node alongside an existing one.
The ROS, Gazebo, and Python commands below run **inside the project container**,
not directly in Windows PowerShell or an unsourced WSL shell.

### 1. Open the container

If `pobs-lake` is already running, skip container creation and enter it from WSL:

```bash
docker exec -it -u ros -e HOME=/home/ros pobs-lake bash
```

If there is no existing container, create one from **WSL**, using the GUI mounts
for both Gazebo and RViz:

```bash
cd /mnt/c/Users/kotha/pobs
export COMPOSE_PROJECT_NAME=pobs-ros2
export DISPLAY="${DISPLAY:-:0}"
docker compose -f compose.yaml -f compose.gui-wslg.yaml run -d \
  --name pobs-lake ros bash -lc 'exec sleep infinity'
docker exec -it -u ros -e HOME=/home/ros pobs-lake bash
```

This assumes the project image has already been built following the root
README. A stopped container with the same name must be removed before creating
another one. The container stays running while its processes are started in
the following shells.

### 2. Build and start Gazebo

In container terminal 1:

```bash
source /opt/ros/lyrical/setup.bash
cd /home/ros/ros2_ws
colcon build --symlink-install
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="/home/ros/ros2_ws/src/pobs_gazebo/models${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
gz sim -r /home/ros/ros2_ws/src/pobs_gazebo/worlds/lake.sdf
```

For headless Gazebo, replace the last command with `gz sim -s -r` followed by
the same world path. If Gazebo is already running, skip that command.

For **each additional terminal**, enter the same container from WSL using
`docker exec` above, then prepare its ROS environment:

```bash
source /opt/ros/lyrical/setup.bash
cd /home/ros/ros2_ws
source install/setup.bash
```

### 3. Start the bridge

In container terminal 2, after sourcing:

```bash
bash /home/ros/ros2_ws/scripts/bridge_pobs.sh
```

Keep it running. The script starts both the sensor/clock bridge and the
depth/pressure publisher; a separate depth publisher is unnecessary.

### 4. Start the visualization launch

In container terminal 3, after sourcing:

```bash
ros2 launch pobs_gazebo trajectory_comparison.launch.py rviz:=true
```

This starts the estimator, comparison node, and configured RViz together.
If the estimator is already running with simulation time, use
`estimator:=false` to avoid a duplicate:

```bash
ros2 launch pobs_gazebo trajectory_comparison.launch.py rviz:=true estimator:=false
```

### Run each node individually instead

Keep Gazebo and the bridge running. In place of the combined launch, run each
command below in its **own sourced container terminal**:

```bash
# Estimator
ros2 run pobs_state_estimator state_estimator --ros-args -p use_sim_time:=true
```

```bash
# Comparison node: publishes both paths and the lake mesh markers
ros2 run pobs_gazebo trajectory_comparison --ros-args -p use_sim_time:=true
```

```bash
# RViz with the saved displays and simulation time
rviz2 -d "$(ros2 pkg prefix --share pobs_gazebo)/rviz/trajectory_comparison.rviz" \
  --ros-args -p use_sim_time:=true
```

### Reset without restarting the container or nodes

In another sourced container terminal:

```bash
python3 /home/ros/ros2_ws/scripts/reset_pobs.py
```

To leave Gazebo paused at the initial state:

```bash
python3 /home/ros/ros2_ws/scripts/reset_pobs.py --paused
```

After a paused reset, click Play in Gazebo, or resume from that container shell:

```bash
gz service -s /world/lake/control \
  --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
  --timeout 3000 --req 'pause: false'
```

These Python commands are what `pobs reset` and `pobs reset --paused` invoke.
The script saves each buoyant link's Volume and CenterOfVolume components,
pauses and resets Gazebo, restores only those components, verifies them, then
resumes unless `--paused` was supplied. It works around the observed Gazebo
10.5 reset dropping buoyancy metadata. Pre-reset poses and velocities are
never restored. Use this script for terminal resets instead of a bare
`reset: {all: true}` service request, which can leave buoyancy missing.

The container, bridge, estimator, comparison node, and RViz remain running;
no rebuild is needed. The backward clock jump clears visualization paths
through the comparison node's reset handler. This resets Gazebo and the
comparison history; it does not restart or explicitly reset the estimator's
internal state. The comparison's settling interval skips its initial stale
samples as described in the implementation notes below.

If buoyancy metadata was already lost through another reset method, start a
healthy world once with `pobs stop` and `pobs sim`, then restart the bridge and
comparison launch. Use `pobs reset` thereafter. If restoration fails after a
reset starts, the script leaves the world paused.

## What to expect in RViz

The saved configuration uses the `lake` fixed frame and shows the lake mesh,
ground-truth path in green, estimated path in orange, and an Actual Robot Pose
arrow subscribed directly to `/model/pobs/pose`. The arrow can appear before
the estimator supplies matched samples. Scroll to zoom toward the AUV.

The comparison node publishes paths once both inputs arrive within the timestamp
tolerance. If estimated odometry is absent, neither path accumulates points.
Check the inputs and outputs from a sourced container shell:

```bash
ros2 node list
ros2 topic echo /model/pobs/pose --once
ros2 topic echo /state/odometry --once
ros2 topic info /visualization/estimated_path
ros2 topic info /visualization/ground_truth_path
```

Both trails clear on reset, then accumulate new samples. The estimated path can
trace the same vertical segment on every run because the current estimator
holds x/y at zero and follows changing depth. A line following the same route
is not by itself evidence that old history survived.

The comparison node publishes `/visualization/lake` as a retained
`visualization_msgs/MarkerArray`, independently of the odometry inputs.
It reuses Gazebo's terrain and water OBJ files in the `lake` frame at unit
scale and zero offset, matching the current `lake.sdf` and terrain model.
If the terrain model's placement changes in Gazebo, update the marker pose
in `make_lake_markers()` to match. Water is faint; uncheck `water` under
the Lake Terrain and Water display's Namespaces to hide it completely.
The geometry is republished periodically and on comparison resets.

## Implementation notes

The node subscribes to both inputs and holds the latest message from each.
Pairs within 50 milliseconds are consumed once and passed to `compare_pair()`.
New arrivals overwrite unmatched messages; this simple approach can miss matches
when messages arrive at different rates or with delays. Buffered matching can
replace it later. Each match adds a pose to both published paths, with at most
2,000 points per path. Both published paths use `lake`. The visualization
node applies the configured horizontal spawn offset to incoming `odom` poses;
the estimator's raw odometry stays unchanged. Incoming estimates already in
`lake` pass through without an offset. Unknown frames are skipped with a
warning. RViz therefore needs no odom transform lookup when its TF buffer is
cleared by a simulation reset. Further validation and error calculation remain
TODOs.

The sensor bridge also publishes `/clock`. A backward clock jump clears both
paths and pending samples and publishes empty paths to clear RViz. Backward
timestamps on either input trigger the same reset as a fallback. Inputs are
expected to arrive in timestamp order; a delayed older sample is treated as
a reset. Path publishers retain their latest message for subscribers joining
after a reset.

After a clock rewind, comparison waits `reset_settle_s` (default 0.25 simulation
seconds) before accepting samples. This avoids drawing the unchanged estimator's
cached pre-reset depth under its first new timestamps. Increase this interval
if sensor startup takes longer; it is a settling allowance, not a proof of
estimator freshness. Previous-run messages ahead of the new clock are rejected.

## Where to work

- `pobs_gazebo/trajectory_comparison.py`: implement the node TODOs.
- `launch/trajectory_comparison.launch.py`: launch the node with optional RViz startup.
- `rviz/trajectory_comparison.rviz`: saved lake view with both colored Path displays.
- `test/test_trajectory_comparison.py`: tests for pairing, path limits, and resets.
- `test/VISUALIZATION.md`: additional test ideas.

## Intended interface

| Direction | Topic | Type |
| --- | --- | --- |
| Input | `/state/odometry` | `nav_msgs/msg/Odometry` |
| Input | `/model/pobs/pose` | `geometry_msgs/msg/PoseStamped` |
| Output | `/visualization/estimated_path` | `nav_msgs/msg/Path` |
| Output | `/visualization/ground_truth_path` | `nav_msgs/msg/Path` |
| Output | TODO: position error topics | TODO: choose and document message types |

Keep ground truth for evaluation only. Both paths need a common frame and
fixed initial reference. Compare positions at matched timestamps, include
signed forward x and vertical z errors, bound history, and clear on reset.
Document axes, units, alignment assumptions, and timestamp tolerance here.

The team confirmed the Gazebo pose topic as the ground-truth input. The
existing `pobs_state_estimator` supplies `/state/odometry`; its source is
unchanged. Our simulation depth publisher supplies its required
`geometry_msgs/PointStamped` on `/pobs/depth`, preserving the Gazebo pose's
timestamp and storing positive depth in `point.z`. The x/y fields are unused
on this measurement topic. `/pobs/pressure` remains `sensor_msgs/FluidPressure`.

The current estimator reports orientation and absolute depth, with x/y fixed
at zero. For display, we define its world-aligned `odom` origin at the known
spawn x/y from `lake.sdf`: (-12, -8). The transform's z offset is zero because
the estimator already reports z = -depth relative to the water surface.
This is a configurable visualization convention, not evidence of horizontal
localization. No measured ground-truth x/y or attitude enters the estimator.
If the team's odom convention changes, update the transform configuration;
disable `publish_odom_transform` when another node already publishes it.
That option controls only the optional TF broadcast; the same configured
offset is always applied to `odom` poses when constructing the lake path.

## Run the focused tests

Inside a sourced project container shell:

```bash
cd /home/ros/ros2_ws
ROS_DOMAIN_ID=87 python3 -m pytest \
  src/pobs_gazebo/test/test_trajectory_comparison.py \
  src/pobs_gazebo/test/test_estimator_integration.py \
  src/pobs_gazebo/test/test_reset_pobs.py -q -p no:cacheprovider
```

The test domain isolates synthetic ROS messages from the running simulation.

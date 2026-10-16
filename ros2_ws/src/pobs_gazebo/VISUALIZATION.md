# POBS visualization scaffold

Starter structure for [issue #3](https://github.com/Isaac-Cabello/POBS/issues/3).
The node subscribes to both inputs and holds the latest message from each.
Pairs within 50 milliseconds are consumed once and passed to `compare_pair()`.
New arrivals overwrite unmatched messages; this simple approach can miss matches
when messages arrive at different rates or with delays. Buffered matching can
replace it later. Each match adds a pose to both published paths, with at most
2,000 points per path. Both inputs are temporarily assumed to share the same
world reference; coordinate transforms, validation, and error calculation
remain TODOs.

The sensor bridge also publishes `/clock`. A backward clock jump clears both
paths and pending samples and publishes empty paths to clear RViz. Backward
timestamps on either input trigger the same reset as a fallback. Inputs are
expected to arrive in timestamp order; a delayed older sample is treated as
a reset. Path publishers retain their latest message for subscribers joining
after a reset.

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
localization system must supply `/state/odometry` for comparisons to occur.

## Build and start the scaffold

The local WSL `pobs` helper uses `/mnt/c/Users/kotha/pobs` as its checkout.
Start the simulator with `pobs sim --headless`, then run `pobs build` to build
inside that same container. Start `pobs bridge` in another terminal for sensor
and clock data, and use `pobs run pobs_gazebo trajectory_comparison` to run the
node. `pobs rviz` opens the saved configuration in the simulator container:
green ground truth, orange estimate, and the `lake` fixed frame. The initial
camera shows the whole lake; scroll to zoom toward the AUV.
The green Actual Robot Pose arrow displays `/model/pobs/pose` directly, so
the robot is visible even before the estimator supplies matched samples.

The same comparison node publishes `/visualization/lake` as a retained
`visualization_msgs/MarkerArray`, independently of the odometry inputs.
It reuses Gazebo's terrain and water OBJ files in the `lake` frame at unit
scale and zero offset, matching the current `lake.sdf` and terrain model.
If the terrain model's placement changes in Gazebo, update the marker pose
in `make_lake_markers()` to match. Water is faint; uncheck `water` under
the Lake Terrain and Water display's Namespaces to hide it completely.
The geometry is republished periodically and on comparison resets.

Follow the repository root README's Docker/WSL workflow. Inside the project container:

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select pobs_gazebo
source install/setup.bash
ros2 launch pobs_gazebo trajectory_comparison.launch.py
```

The node publishes paths once both inputs arrive within the timestamp tolerance.
If estimated odometry is absent, neither path accumulates points.
Open the configured node and RViz together with:

```bash
ros2 launch pobs_gazebo trajectory_comparison.launch.py rviz:=true
```

Or open just the saved RViz view:

```bash
rviz2 -d "$(ros2 pkg prefix --share pobs_gazebo)/rviz/trajectory_comparison.rviz"
```

Run the focused tests inside the project container:

```bash
ROS_DOMAIN_ID=87 python3 -m pytest src/pobs_gazebo/test/test_trajectory_comparison.py -q
```

The test domain isolates synthetic messages from the running simulation.

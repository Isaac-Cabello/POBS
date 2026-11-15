# POBS Master Planner V1.1: simulation-only XY waypoint test

Added files:
- `pobs_master_planner/gazebo_pose_to_odom.py`: converts Gazebo `/model/pobs/pose` into a local-origin XY `nav_msgs/Odometry` on `/planner/sim_odometry`.
- `launch/planner_gazebo_dry_run.launch.py`: runs the pose adapter and planner, remaps its input to sim odometry, and disables calibration **only for this dry-run launch**.
- Modified `setup.py` registers the new executable and launch file.

## Run

Do not launch both planners simultaneously. Stop the old `master_planner.launch.py` launch first.
This test remaps controller target output to `/planner/dry_run_target_pose`, isolating it from any real controller. It does not send motor or thruster commands.

1. Keep a Gazebo simulation running that publishes `/model/pobs/pose`.
2. In another Docker terminal, bridge its pose:

   `ros2 run ros_gz_bridge parameter_bridge '/model/pobs/pose@geometry_msgs/msg/Pose[gz.msgs.Pose'`

   First verify `gz topic -i -t /model/pobs/pose` reports `gz.msgs.Pose`.

3. Build and run:

   ```bash
   cd /home/ros/ros2_ws
   colcon build --packages-select pobs_master_planner --symlink-install
   source install/setup.bash
   ros2 launch pobs_master_planner planner_gazebo_dry_run.launch.py
   ```

4. Check `/planner/status` until `current_xy` is numeric and state is `READY`.
5. Send a goal (meters from the starting position):

   ```bash
   ros2 topic pub --once /planner/goal geometry_msgs/msg/PoseStamped \
     '{header: {frame_id: odom}, pose: {position: {x: 1.0, y: 0.5, z: 0.0}, orientation: {w: 1.0}}}'
   ```

   Status should become `NAVIGATING`; `/planner/dry_run_target_pose` will publish the target; the robot will **not move** because no motion controller is attached.

## Cautions

- The simulation bypass is not a calibration fix. Leave normal `require_calibration: true` in `config/planner.yaml`.
- The pose adapter is simulation ground truth and should never replace the estimator on physical POBS.
- The original planner publishes `z=-3.0` placeholder in its target; V1 does **not** control depth or bottom clearance.
- The existing tilted `lake.sdf` may still show `UNSAFE_TILT` on the calibration node. This dry-run intentionally isolates waypoint logic from that earlier equilibrium issue.
- For a subsequent *moving* waypoint test, implement a separate position controller, thruster allocation and failsafes, preferably using the previously tested balanced world/model.

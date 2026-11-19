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

---

## Session 2–3: Powered XY Navigation (November 2026)

The original dry-run procedure above remains valid. The powered simulation
uses `planner_gazebo_powered.launch.py` and the balanced Gazebo world.

### Components
- Master Planner: waypoint supervision and goal completion.
- XY Controller: heading correction and velocity-aware braking.
- Simulation Safety Gate: checks freshness, tilt, boundary, and controller state.
- Gazebo Pose Adapter: converts simulation pose to relative XY odometry.

### Powered simulation setup
1. Start `lake_balance_test.sdf` in Gazebo.
2. Bridge `/model/pobs_balance_test/pose` using `ros_gz_bridge`.
3. Bridge Gazebo thruster commands for thrusters 4 and 5.
4. Launch `planner_gazebo_powered.launch.py`.
5. Publish a PoseStamped waypoint to `/planner/goal`.
6. Verify the planner and controller statuses before arming.

Arm: ros2 param set /sim_thruster_safety enabled true
Disarm: ros2 param set /sim_thruster_safety enabled false

### Controller and safety limits
- Goal tolerance: 0.20 m
- Goal settling time: 1.0 s
- Maximum settling speed: 0.03 m/s
- Simulation thrust cap: 0.5 N
- Safety boundary: 3.0 m from relative odometry origin
- Safety tilt limit: 10 degrees

The controller supports velocity-aware braking and recovery from invalid
velocity measurements. The planner retains reached goals and resumes
navigation when the robot moves outside the acceptance criteria.

### Verified Session 3 results
Waypoint 1: target (0.5, 0.5), final error 0.058 m.
Waypoint 2: target (0.0, 0.5), final error 0.065 m.

Both waypoints reached GOAL_REACHED without restarting the planner.
The controller subsequently became INACTIVE, and the safety gate was
confirmed DISARMED with zero commanded thrust.

### Automated testing
Run inside Docker:

colcon test --packages-select pobs_master_planner --event-handlers console_direct+

Regression tests cover velocity recovery, controller behavior, goal
settling, overshoot recovery, and simulation safety boundaries.

### Limitations
- Simulation only; do not use this gate as a hardware safety controller.
- Relative XY origin resets when the Gazebo pose adapter restarts.
- Gazebo ground-truth position is not equivalent to physical localization.
- This system does not integrate sonar or control depth.
- Do not run competing thruster publishers from the stabilizer/allocator.
- The safety gate is not an independent watchdog.

## Session 4 — Integrated Stabilizer and Allocator Testing

Verified:
- Master Planner -> XY Controller -> Stabilizer communication.
- Stabilizer -> Thruster Allocator wrench communication.
- Navigation command timeout (0.5 s).
- Thruster allocator timeout (0.25 s).
- Simulated thruster actuation with conservative force limits.
- Positive yaw response to a manually commanded yaw torque.
- Yaw-only test relay filtering out vertical and pitch commands.

Known limitations:
- Full powered waypoint navigation has not been validated.
- Stabilizer yaw torque increased without an active navigation command.
- Mock depth remains fixed at 3 m and does not represent actual Gazebo depth.
- The temporary yaw-only relay is a simulation test utility, not part of
  the production control architecture.

Next steps:
- Diagnose stabilizer yaw PID behavior.
- Integrate actual simulated depth feedback.
- Validate closed-loop powered waypoint navigation.

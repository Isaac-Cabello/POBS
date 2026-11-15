# POBS Master Planner — recovered package (November 2026)

This package restores the files used in the conversation, including all four
executables and the original configuration/launch files.

## Contents

- `master_planner.py` — receives XY goals, monitors `/state/odometry` and
  `/calibration/status`, publishes `/planner/status`, `/planner/goal_reached`,
  and `/controller/target_pose`.
- `gazebo_pose_to_odom.py` — **simulation only**, maps Gazebo Pose to relative
  `/planner/sim_odometry`.
- `xy_controller.py` — heading/distance proportional controller publishing
  normalized **dry-run** commands on `/planner/dry_run/thruster_4` and `_5`.
- `sim_thruster_safety.py` — **simulation only**, disarmed by default, converts
  normalized values to limited newton commands after safety checks.
- `config/planner.yaml` — planner settings (real calibration remains required).
- `launch/master_planner.launch.py` — original planner launch.
- `launch/planner_gazebo_dry_run.launch.py` — original planner-only simulation
  test launch (does not start the XY controller/safety node).

## Restore

Copy this **entire** `pobs_master_planner` directory into:

    POBS-ROS2/ros2_ws/src/pobs_master_planner/

From the **Docker** ROS2 terminal (not macOS):

    cd /home/ros/ros2_ws
    colcon build --packages-select pobs_master_planner --symlink-install
    source install/setup.bash
    ros2 pkg executables pobs_master_planner

Expected executables: `master_planner`, `gazebo_pose_to_odom`,
`xy_controller`, `sim_thruster_safety`.

## What was intentionally not included

The downward-sonar additions to `sim_thruster_safety.py` were explicitly undone
in VS Code before the folder disappeared. Accordingly, this recovered version
is the last complete **pre-sonar** gate, and `clearance_m` is **not** part of
its status. The real sonar bridge is separate and can be re-integrated later.

## Safety and usage notes

- Keep `sim_thruster_safety` `enabled:=false` until independent safety checks
  are completed. The gate is **not an independent watchdog**. If it or its
  bridge dies after a nonzero command, thrust can remain latched in Gazebo.
- The original standalone planner outputs a placeholder Z (`-3.0` m), not a
  measured depth target. The XY controller ignores Z.
- Only use `gazebo_pose_to_odom` for simulation, not physical underwater tests.
  IMU-only XY localization is not reliable enough to certify waypoint position.
- The dry-run launch bypasses calibration intentionally; this bypass must not
  be used to enable physical thrusters.
- If testing the balanced model `pobs_balance_test`, ensure the Gazebo pose
  bridge and adapter use `/model/pobs_balance_test/pose` rather than the
  original `/model/pobs/pose`. The Thruster plugins retained the namespace
  `/model/pobs/joint/thruster_{4,5}_joint/cmd_thrust` in the prior test.

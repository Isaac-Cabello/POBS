# Tests to implement

- Identical trajectories produce zero error after initial alignment.
- A known added displacement produces the expected signed x/z and total error.
- Different initial positions and orientations align to the intended reference.
- Timestamp matching handles different publishing rates and arrival delays.
- Missing or stale samples do not produce misleading comparisons.
- Trajectory and synchronization buffers stay within their configured limits.
- Simulation reset clears paths, buffers, and alignment, including the RViz display.
- A ROS integration test verifies subscriptions, output types, frames, and stamps.

No tests or comparison behavior are implemented in this scaffold.

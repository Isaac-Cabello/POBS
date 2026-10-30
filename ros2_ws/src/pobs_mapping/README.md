# Accumulated ground-truth sonar mapping

The mapper subscribes to all four `/pobs/sonar/sonar_*/range` topics and
`/model/pobs/pose`. It does **not** subscribe to `/state/odometry`.

At initialization it reads `model.sdf`, resolves each sensor's mounting pose
through its parent link and any named SDF frames, and caches its rotation matrix
and offset relative to the model origin. Ground truth is the world pose of that
model origin. Each finite in-range reading is projected along sensor +X using
the ground-truth pose at its acquisition timestamp. Translation is interpolated
linearly and orientation with quaternion SLERP; no pose extrapolation is used.

Every accepted point remains in memory for the entire run. There is no rolling
window, voxel filtering, downsampling, or point-count cap. A configurable 5 Hz
publisher sends the **entire accumulated cloud** on `/pobs/mapping/points` as
`PointCloud2`, with reliable/transient-local QoS so RViz can open after collection
starts and still receive the map. The separate pending-reading queue and pose
history are bounded; their limits do not remove any already accepted map points.

## Run

Inside the project ROS container after building `pobs_sonar`, `pobs_mapping`,
and `pobs_visualization`:

```bash
# Start Gazebo and sensors separately first.
pobs sensors source:=simulation
pobs rviz point_cloud_mapping
```

If `start_lake_sim.sh` already started the existing sensor bridges, use
`pobs sensors source:=simulation existing_sensors:=false` instead.

To show both independently toggleable RViz groups:

```bash
pobs rviz state_estimation point_cloud_mapping
```

Run the node without RViz:

```bash
ros2 run pobs_mapping point_cloud_mapper --ros-args \
  -p model_path:=$HOME/ros2_ws/src/pobs_gazebo/models/pobs/model.sdf \
  -p world_frame:=lake
```

Ground truth must use `world_frame` (default `lake`). Empty source frame IDs are
accepted only under the assumption that the configured `/model/pobs/pose` source
is already in that world frame. Nonempty mismatching frames are rejected.
Sensor frame IDs must match `sonar_left_link`, `sonar_right_link`,
`sonar_middle_link`, and `sonar_depth_link`.

## Controls and limits

```bash
ros2 service call /pobs/mapping/clear std_srvs/srv/Trigger '{}'
ros2 topic echo /pobs/mapping/points --field width --qos-durability transient_local
```

Points disappear only after manual clear, simulation clock rollback, or process
exit. Stopping/restarting RViz does not clear a running mapper. Restarting the
mapper starts a new map; there is no on-disk persistence yet. A backward pose
jump of more than 0.5 s also resets the map when no `/clock` source is available.

Other parameters: `pose_topic`, `publish_rate_hz` (5), `pose_history_s` (10),
`max_pose_gap_s` (0.5), `pending_timeout_s` (1),
`pose_reset_threshold_s` (0.5), and `max_pending_readings` (1000).

The cloud contains hits from **all four sensors**, including forward obstacles.
It is not automatically classified as bare seafloor. Points are beam-centerline
approximations; the current single-ray Gazebo sensors match that model, while
real wide-beam sonar has angular uncertainty. Long runs retain all points, so
memory and full-cloud publication cost grow with the collected data.

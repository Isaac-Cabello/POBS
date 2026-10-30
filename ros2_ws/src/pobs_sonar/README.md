# SEN0599 mapping sonar driver

Four instances of one ROS 2 driver, each publishing one measured distance in
meters as `sensor_msgs/msg/Range.range`:

| Node | Topic | Default serial port | Frame |
| --- | --- | --- | --- |
| `/pobs/sonar/sonar_left` | `/pobs/sonar/sonar_left/range` | `/dev/ttyUSB0` | `sonar_left_link` |
| `/pobs/sonar/sonar_right` | `/pobs/sonar/sonar_right/range` | `/dev/ttyUSB1` | `sonar_right_link` |
| `/pobs/sonar/sonar_middle` | `/pobs/sonar/sonar_middle/range` | `/dev/ttyUSB2` | `sonar_middle_link` |
| `/pobs/sonar/sonar_depth` | `/pobs/sonar/sonar_depth/range` | `/dev/ttyUSB3` | `sonar_depth_link` |

The downward sonar uses the shared driver/adapter at the same robot-center
mount. Its original simulation measurement settings are retained; its public
topic is now `/pobs/sonar/sonar_depth/range`. This package does not add a mapper.

## Downward sensor compatibility

The original downward simulation keeps its 0.15-30 m range, 10 Hz updates,
1 cm resolution, 1 cm Gaussian noise, and single downward ray at an 8 cm
offset. Only its transport changes: the raw Gazebo scan is now
`/pobs/sonar/sonar_depth/scan`, and ROS consumers use `sensor_msgs/msg/Range`
on `/pobs/sonar/sonar_depth/range` instead of `LaserScan` on `/pobs/sonar`.
Read `.range` in meters instead of `.ranges[0]`. Source timestamps and explicit
infinite no-hit/too-close values are preserved for the simulated downward ray.

The physical SEN0599 has a maximum range of 6 m; it cannot reproduce the
original 30 m simulation. Its downward hardware profile retains the 0.15 m
minimum but advertises a 6 m maximum and its physical 16-degree sensing angle.
The four UART instances share a 400 ms cycle, staggered by 100 ms, giving about
2.5 readings/second each. This hardware rate is separate from simulated 10 Hz.

`/pobs/depth` remains water depth from vehicle pose, not bottom clearance.
It now publishes `geometry_msgs/msg/PointStamped` with positive depth in
`point.z` and the pose timestamp, matching the existing state estimator's
subscriber (the previous Float64 publisher was incompatible). The formula
`max(0, -robot_z)` and pressure formula are unchanged. No node in this branch
subscribed directly to the old `/pobs/sonar` LaserScan; the stabilizer currently
does not subscribe to depth or sonar.

## Robot mounting

The Gazebo `pobs` model includes visible housings and front mounting brackets.
Face poses relative to `base_link` (+X forward, +Y left, +Z up):

| Sensor | X, Y, Z (m) | Direction |
| --- | --- | --- |
| `sonar_left` | 0.43, +0.15, -0.02 | Forward and down 45 degrees |
| `sonar_right` | 0.43, -0.15, -0.02 | Forward and down 45 degrees |
| `sonar_middle` | 0.44, 0, 0 | Forward |
| Existing downward sonar | 0, 0, -0.08 | Straight down |

Offsets are provisional, based on the existing 0.8 x 0.4 m chassis. Replace
them with measured face positions before using real data for mapping. The
launch publishes static transforms from `base_link` to `sonar_left_link`,
`sonar_right_link`, `sonar_middle_link`, and `sonar_depth_link` for both sources.
Mount definitions in `pobs_sonar/mounting.py` must match `models/pobs/model.sdf`;
the geometry test checks this.

The three front Gazebo sensors are idealized single-ray proxies with 0.05-6 m range,
10 Hz updates, and 1 cm Gaussian noise. They do not simulate the sonar's full
beam footprint, acoustic scattering, or interference. The visual housings do
not add mass, collision geometry, or displaced volume to the existing vehicle.

With Gazebo already running, select simulated ranges instead of UART hardware:

```bash
pobs sensors source:=simulation
```

If `start_lake_sim.sh` already started the existing sensor pipeline:

```bash
pobs sensors source:=simulation existing_sensors:=false
```

Both sources publish the same four `Range` topics. Simulation scans also appear
on `/pobs/sonar/sonar_left/scan`, `/pobs/sonar/sonar_right/scan`, and
`/pobs/sonar/sonar_middle/scan`, and `/pobs/sonar/sonar_depth/scan`.
The adapter preserves the simulator timestamp
and rejects missing or out-of-range returns. A launch selects exactly one source.

## Activate all sensor topics

After rebuilding the Docker image (to install the `pobs` command) and building
the package, run inside the container:

```bash
pobs sensors
```

This starts the existing `scripts/bridge_pobs.sh` pipeline for `/pobs/imu`,
`/model/pobs/pose`, `/pobs/depth`, and `/pobs/pressure`, together
with the left/right/middle/downward UART drivers listed above. Gazebo must already be
running for the existing simulation sensors to provide data; the four
mapping sonars use physical sensors by default, or mounted Gazebo sensors with
`source:=simulation`. No synthetic ranges are emitted for missing hardware,
and this command does not start Gazebo or the stabilizer.
The depth adapter publishes stamped water depth for the existing state estimator.

If `start_lake_sim.sh` has already started the bridge and depth adapter, use:

```bash
pobs sensors existing_sensors:=false
```

Do not run two copies of the same sensor launch. Ctrl+C stops the processes
started by this command. To override serial settings, append
`config:=/absolute/path/sonars.yaml`. `POBS_WORKSPACE` defaults to `~/ros2_ws`.

Without rebuilding the image, the source command also works:

```bash
bash ~/ros2_ws/scripts/pobs sensors
```

## Hardware and build

Use four independent UART ports (or four USB-to-UART adapters) with a common
ground. Use the correct logic-level interface for the sensor; a power supply
rating does not establish UART logic compatibility. Each connection runs at
115200 baud, 8N1. Configure `config/sonars.yaml` with your device paths, preferably
stable `/dev/serial/by-id/...` paths. Keep each port assigned to just one node.

Follow the repository's WSL/Linux container workflow. Expose the four serial
devices to the container and grant the container user access to them. Windows
USB devices must first be made available in WSL; the current base Compose file
does not configure serial passthrough. Do not add broad privileged access.

Inside the project ROS container:

```bash
cd ~/ros2_ws
rosdep install --from-paths src/pobs_sonar --ignore-src -r -y
colcon build --symlink-install --packages-select pobs_sonar
source install/setup.bash
ros2 launch pobs_sonar sonars.launch.py
```

To use a custom configuration without editing the installed package:

```bash
ros2 launch pobs_sonar sonars.launch.py config:=/absolute/path/sonars.yaml
ros2 topic echo /pobs/sonar/sonar_left/range --field range --qos-reliability best_effort
```

One standalone instance can also be started with:

```bash
ros2 run pobs_sonar sen0599_driver --ros-args \
  -p port:=/dev/ttyUSB0 -p frame_id:=sonar_left_link \
  -p topic:=/pobs/sonar/sonar_left/range
```

## Measurement behavior

- Sends `0x55`; decodes `FF HIGH LOW CHECKSUM`, verifies the byte-sum checksum,
  and converts millimeters to meters. Handles partial frames and stream noise.
- Publishes only readings within 0.05-6.0 m. Zero, corrupt responses, and timeouts
  produce no range message; old measurements are never republished as new ones.
- Uses sensor-data QoS (best effort), a unique frame, and a receipt timestamp.
  The device does not report an acquisition timestamp or echo confidence.
- Uses the advertised 16-degree maximum angle as a configurable field of view.
- Reconnects after missing devices or serial errors. Reads are nonblocking;
  writes have a 5 ms timeout, and an echo has a 50 ms deadline by default.
- Four processes on the **same Linux host** use a shared monotonic-clock cycle.
  Defaults allow one trigger per sensor per 400 ms, staggered by 100 ms (about
  2.5 readings/second per sensor). Late trigger windows are skipped. Keep periods
  identical and offsets separated by more than the trigger window plus response
  timeout. Validate interference on hardware; reverberation can require longer
  spacing. This is software scheduling, not a hardware synchronization guarantee.

The output is the target distance selected by the sensor. DFRobot does not
document the echo-selection algorithm, so the driver cannot guarantee that it
is always the geometrically closest reflector or independently choose echoes.

Protocol: https://wiki.dfrobot.com/sen0599/docs/18113

Trigger command example: https://wiki.dfrobot.com/sen0599/docs/18114

## Validation

Run the tests inside the project container:

```bash
colcon test --packages-select pobs_sonar --event-handlers console_direct+
colcon test-result --verbose
```

Tests cover corrupt/partial UART frames, invalid distances, reconnect behavior,
and four scheduled sensor instances publishing into independent topics. Tests
also verify the old downward simulation specs and the depth/estimator interface.
Hardware checks still need four connected SEN0599 sensors submerged in water.

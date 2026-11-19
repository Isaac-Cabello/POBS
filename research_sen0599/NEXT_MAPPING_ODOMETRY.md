# Next mapping and odometry work for POBS

Research date: October 30, 2026. This is a proposal, not an implemented estimator.
The default recommendation assumes the existing four SEN0599s, IMU and separate
water-depth measurement. Hardware alternatives are discussed separately.

## Recommendation

Implement an uncertainty-aware bathymetric map, then a terrain-aided localization
prototype against a frozen reference map. This is the most defensible way to
test whether our existing sonar arrangement can correct horizontal drift.
For reliable odometry while exploring previously unmapped terrain, prioritize
an independent motion measurement: a bottom-tracking DVL, or a downward camera
with visual-inertial estimation when visibility and bottom texture permit it.

A Kalman filter alone does not add missing horizontal information. Four range
sensors also do not constitute a DVL: our devices measure distance, whereas a
DVL uses Doppler measurements to estimate velocity.

## What the current code establishes

- The mapper already resolves SDF mounts and interpolates timestamped Gazebo
  ground truth. It accumulates centerline endpoints; it does not estimate motion.
- The estimator integrates gravity-compensated IMU acceleration for x/y and uses
  absolute water depth for z. It does not currently estimate IMU biases or propagate
  a full state covariance; published covariance entries are fixed.
- Left/right have the same 45-degree down-forward direction and zero yaw. Their
  lateral separation is 0.30 m. They are parallel, rather than a sideways fan.
- Hardware profiles currently schedule about 2.5 measurements/s per sensor.
  Simulation uses 10 Hz, and the legacy downward simulation reaches 30 m, versus
  the actual sensor's 6 m maximum. These differences must be included in tests.

## Why point accumulation is not yet a physical surface reconstruction

DFRobot specifies 5–600 cm range and an 8–16 degree sensing angle. A range reading
does not tell us where within that angle the echo originated. The official ROS
Range definition explicitly describes this angular ambiguity. A beam-center
endpoint remains useful for display, but is an approximation for real sonar.
[DFRobot specifications](https://www.dfrobot.com/product-2760.html),
[ROS Range definition](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Range.msg).

For scale: if 16 degrees is treated as the full cone angle, the geometric width
at 3 m is approximately 2 * 3 * tan(8 degrees) = 0.84 m. This is an illustrative
footprint, not a claimed uniform detection pattern or measured mapping error.
The actual acoustic response needs underwater calibration.

On an infinite horizontal plane, translating sideways while preserving attitude
and altitude leaves these bottom ranges unchanged. Consequently, horizontal
translation cannot be inferred from those measurements alone. On a single planar
slope, motion along a contour remains ambiguous. These are geometric properties
of our measurement arrangement, not failures that smoothing can resolve.

## Milestone 1: improve the map and its measurement model

Add a derived 2.5-D map to pobs_mapping while retaining the existing complete
point cloud and raw timestamped observations. Each horizontal cell should hold
height, uncertainty, observation count and last observation time. Unobserved
cells remain unknown; interpolated regions must be identifiable as inferred.
Use a separate obstacle representation for the forward sonar and vertical faces,
because a single-height grid cannot represent a wall or overhang faithfully.

Start with a configurable 0.2–0.3 m cell size for this small test area. This is a
storage/display choice, not a promise of that spatial accuracy. Propagate pose,
range and beam-angle uncertainty. Repeated overlapping readings share pose errors
and must not be treated as arbitrarily many independent high-confidence samples.
An angular uncertainty region represents possible echo origins; do not mark an
entire cone or arc as an observed occupied surface from one scalar return.

Calibrate mounts, range bias, repeatability and latency underwater. Preserve
acquisition times when filtering. Gate implausible returns using predicted range
and uncertainty, while testing that real seabed steps and objects survive gating.
Use a finite-beam, nearest-visible-surface simulation approximation and inject
dropouts, biases and realistic rates. Do not claim that it reproduces acoustic
scattering or multipath without physical validation.

This follows the uncertainty treatment of probabilistic elevation mapping.
ANYbotics' elevation_mapping is a useful algorithm reference but its documented
installation is ROS 1/catkin. The separate grid_map library has ROS 2 branches,
layered elevation/variance storage and RViz visualization. It supplies map
infrastructure, not a complete sonar inverse sensor model or odometry estimator.
[Probabilistic mapping reference and code](https://github.com/ANYbotics/elevation_mapping),
[ROS 2 grid_map](https://github.com/ANYbotics/grid_map/tree/ros2).
Compatibility with our exact Lyrical environment still requires a build check.

As a separate geometry experiment, compare existing parallel beams with left/right
yaw angles of roughly +30/-30 degrees, retaining 45-degree downward pitch. On a
level bottom 2 m below the sensor faces, ideal centerline hit separation would
grow from approximately 0.30 m to 2.30 m. This is a calculated proposal; it needs
occlusion, footprint, interference and obstacle-coverage tests before changing
the actual mounts.

## Milestone 2: use the existing ranges to test terrain-aided localization

Create a pobs_localization node alongside the current estimator. Begin with a
particle filter over horizontal position and heading, predicted using inertial
dead-reckoning increments and uncertainty. Use measured attitude and separate
water depth for vertical geometry. For each candidate pose, predict sonar ranges
against a frozen map and weight candidates by range residuals, calibrated sensor
uncertainty, map uncertainty and an outlier model. Incorporate each asynchronous
measurement at its own timestamp.

Use the three downward-looking sensors for bathymetry. Use the forward sensor
only when the reference representation includes its possible obstacle surfaces.
Check terrain information before accepting a strong correction: on flat,
repetitive or insufficiently observed terrain, retain uncertainty rather than
forcing a position fix. A particle filter is an initial design choice because
several different positions can explain sparse ranges; it is not a guarantee
of convergence or a substitute for a good motion prior.

Single-beam terrain-aided navigation is supported by published work, including
particle-filter methods. These methods depend on a prior bathymetric map and
terrain characteristics; their field results should not be transferred directly
to our low-cost sensors and short-range geometry.
[Salavasidis et al., Journal of Field Robotics, 2019](https://onlinelibrary.wiley.com/doi/10.1002/rob.21832),
[Robust fusion TAN with a single-beam echo sounder, Ocean Engineering, 2023](https://www.sciencedirect.com/science/article/pii/S0029801823019947),
[Terrain matching and flat/repetitive-terrain limitations, 2018](https://onlinelibrary.wiley.com/doi/10.1155/2018/3717430).

For the first simulation test, export and freeze a reference survey made with the
existing truth-based mapper, or explicitly provide a known terrain model. Then
localize on later passes. This is known-map localization. It is not mapping an
unknown environment without external information. A map made from the same noisy
estimate cannot simply be fed back as independent truth; unknown-map SLAM needs
joint estimation or appropriately correlated submaps and loop closures later.

## Milestone 3: improve general odometry if additional hardware is possible

### DVL + IMU + pressure/depth

This is the strongest general odometry recommendation for turbid water when
bottom lock is available. Fuse bottom-relative velocity with inertial prediction
and water depth. Use the sonar for mapping and altitude constraints. Maintain
the DVL lever arm and rotation; check velocity validity/covariance and explicitly
handle bottom-lock loss. Water tracking measures velocity relative to water,
which is different from seabed-relative motion in currents.

Water Linked publishes an official ROS 2 driver for A50/A125 devices. This is a
candidate integration, not a checked Lyrical build or a hardware purchase
recommendation independent of vehicle budget, altitude and depth requirements.
[Official driver](https://github.com/waterlinked/waterlinked_dvl),
[Manufacturer integration resources](https://docs.waterlinked.com/dvl/software-resources/),
[Velocity validity and bottom lock](https://docs.waterlinked.com/dvl/how-to-diagnose/),
[Water tracking distinction](https://docs.waterlinked.com/dvl/water-tracking/).

### Downward camera + IMU + sonar

When visibility, illumination and seabed texture are adequate, visual tracking
provides lateral motion and sonar can help constrain scene depth/scale. Camera
calibration must be done for the underwater housing/optics, with accurate sensor
extrinsics and timestamps. A tilted wide-beam return is not automatically the
depth of an arbitrary tracked image feature, especially over rocks or slopes.

VIMS is a relevant research example: it augments visual-inertial estimation with
single-beam sonar under a locally planar-bottom assumption. Its full loop-closure
system additionally uses a calibrated magnetometer and an external magnetic-field
coil, so the complete published system is not a drop-in camera/sonar solution.
Do not promise its reported performance for POBS.
[VIMS, IROS 2025 author paper](https://arxiv.org/html/2506.15126v1).

An error-state Kalman filter with accelerometer/gyro bias states is a suitable
longer-term fusion architecture. robot_localization is a useful ROS integration
baseline for conventional pose/velocity/IMU inputs; its standard configuration
does not implement our nonlinear range-to-map updates or a full IMU bias-state
inertial solution. Adding an EKF with only IMU and depth cannot make unobserved
horizontal motion observable.
[robot_localization sensor configuration](https://github.com/cra-ros-pkg/robot_localization/blob/rolling-devel/doc/configuring_robot_localization.rst).

## Test and integration contract

Keep the existing truth mapper as a benchmark. Add selectable estimated-pose
mapping and separate topics, so the two outputs cannot be confused. The estimator
must not subscribe to /model/pobs/pose. A simulated pressure/depth sensor can of
course be generated from physical simulator state, but should carry realistic
noise/bias instead of providing a perfect hidden pose observation.

Retain raw ranges and sensor-to-keyframe associations. If a later localization
correction moves an earlier pose, the estimated map must be reprojected or its
submap moved. Permanently baking endpoints into world coordinates prevents
proper correction of an accumulated map.

Record reproducible bags and compare identical trajectories: flat floor, a slope,
distinctive bumps/objects, repeated features, crossing/revisited tracks, loss of
bottom return, and changing attitude. Include the physical 6 m maximum and
current 2.5 Hz hardware schedule. Use controlled survey motion and altitude;
the existing uncontrolled buoyant rise is not a representative survey trajectory.

Measure horizontal/heading error against held-out truth, drift per distance,
range residuals, map height/position error, coverage, covariance consistency,
dropout recovery, runtime and memory. A successful flat-floor test should show
that the filter recognizes weak horizontal information, not invented certainty.

## Other resources screened

- [Four-SBES terrain-following thesis implementation](https://github.com/fabiogueunige/TerrainFollowingThesis):
  related altitude/slope estimation, but MATLAB simulation with a different
  cross-pattern rig and an ideal DVL. It is not a ROS 2 sonar-only odometry plugin.
- [Low-cost single-beam SONAR localization in AUVs](https://www.sciencedirect.com/science/article/pii/S0029801826043283):
  publisher-indexed preview has a November 15, 2026 issue date. Abstract only was
  accessible in this review. Its confined-environment surge/sway/heave arrangement
  and external-sonar tracking differ from our parallel down-forward beams; its
  reported accuracy is not evidence for equivalent open-seafloor performance.
- [Sparse point-cloud bathymetry paper](https://arxiv.org/abs/2210.10263):
  uses side-scan imagery, so it is not a direct algorithm for our scalar ranges.

The implementation order recommended for current hardware is measurement realism
and uncertainty mapping, then known-map terrain-aided localization. For exploration
without a prior map, prioritize an independent motion sensor before full SLAM.

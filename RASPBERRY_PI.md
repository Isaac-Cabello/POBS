# Raspberry Pi setup

The repository currently uses ROS 2 Lyrical. Use a 64-bit ARM Ubuntu 26.04
installation and install ROS 2 Lyrical using the official instructions:
https://docs.ros.org/en/lyrical/Installation/Ubuntu-Install-Debs.html
Choose `ros-lyrical-ros-base` for a headless Pi. ROS documents Ubuntu 26.04
arm64 as a Tier 1 platform:
https://github.com/ros2/ros2_documentation/blob/rolling/source/Releases/lyrical/supported-platforms.rst

`main` includes the estimator, BNO055 driver submodule and allocator. Sonar,
mapping and the combined control launcher are feature-branch components;
integrate those packages before using the sonar section below.

These instructions target native Linux on the Pi. The Windows/WSL `pobs`
wrapper starts a Docker simulator and is not the Pi hardware launcher.

## Install dependencies

Run from the repository root after configuring the ROS apt repository:

```bash
sudo apt update
sudo apt install -y ros-lyrical-ros-base python3-colcon-common-extensions \
  python3-rosdep python3-venv python3-pip python3-numpy python3-scipy \
  python3-serial python3-yaml
source /opt/ros/lyrical/setup.bash
```

Initialize rosdep once per Pi (skip `init` if already initialized):

```bash
sudo rosdep init
rosdep update
```

Install the hardware ROS packages' dependencies and build just those packages:

```bash
git submodule update --init --recursive
cd ros2_ws
rosdep install --from-paths src/pobs_state_estimator src/pobs_stabilizer \
  src/bno055 --ignore-src --rosdistro lyrical -r -y
colcon build --symlink-install --base-paths \
  src/pobs_state_estimator src/pobs_stabilizer src/bno055
source install/setup.bash
```

Explicit base paths avoid requiring Gazebo/RViz source packages for this build.
The sonar/stabilizer manifests currently also declare `ros_gz_bridge`, so
rosdep installs that dependency even when using UART hardware.

`requirements.txt` lists the non-ROS Python dependencies. The apt packages
above supply them without changing Ubuntu's managed Python. If you need pip,
run these commands from the repository root with the ROS environment sourced:

```bash
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install -r requirements.txt
```

Do not use `sudo pip` or `--break-system-packages`. `rclpy`, ROS message
packages, TF, colcon and Gazebo are not installed by this requirements file.

## Run physical sonars (requires the feature-branch sonar package)

After integrating `src/pobs_sonar`, install its dependencies and build it:

```bash
rosdep install --from-paths src/pobs_sonar --ignore-src -r -y
colcon build --symlink-install --base-paths src/pobs_sonar
source install/setup.bash
```

Give your account serial-device access, then log out and back in:

```bash
sudo usermod -aG dialout "$USER"
```

Set the four port names in `ros2_ws/src/pobs_sonar/config/sonars.yaml` to the
Pi's actual devices, preferably `/dev/serial/by-id/...` paths. Then from
`ros2_ws`:

```bash
source /opt/ros/lyrical/setup.bash
source install/setup.bash
ros2 launch pobs_sonar sonars.launch.py source:=hardware \
  config:="$(pwd)/src/pobs_sonar/config/sonars.yaml"
```

In another terminal, source the same setup files and check:

```bash
ros2 topic echo /pobs/sonar/sonar_left/range --field range \
  --qos-reliability best_effort
```

## State estimation and allocation

The estimator also needs a physical IMU publisher on `/pobs/imu` and an
absolute water-depth publisher on `/pobs/depth` (`PointStamped`, positive
depth in `point.z`). Sonar bottom clearance does not supply water depth.
This checkout's Gazebo bridge/depth adapter supplies simulated measurements;
`main` includes a BNO055 driver submodule and hardware launch. Configure its
UART port in `src/pobs_state_estimator/config/bno055_pobs.yaml`, then run
`ros2 launch pobs_state_estimator bno055_pobs.launch.py` to start the IMU and
estimator together. A physical absolute water-depth publisher is still needed.

Once those real sensor publishers are running:

```bash
ros2 run pobs_state_estimator state_estimator --ros-args -p use_sim_time:=false
```

The existing allocator can run on the Pi independently of Gazebo. From
`ros2_ws`, in another sourced terminal:

```bash
ros2 run pobs_stabilizer thruster_allocator --ros-args \
  -p sdf_path:="$(pwd)/src/pobs_gazebo/models/pobs/model.sdf" \
  -p force_cap:=10.0
```

It consumes continuous `WrenchStamped` messages on `/cmd_wrench` with
`header.frame_id: base_link`, and outputs per-thruster `Float64` forces in
newtons. It zeros commands after 0.25 seconds without input. The 10 N cap is
an initial simulation setting, not a hardware rating. A physical motor/ESC
driver must convert these force topics into calibrated hardware commands;
the ROS-to-Gazebo bridge only drives simulation. `control.launch.py` is a
simulation launcher and uses `~/ros2_ws` for model paths.

## Validation status

Dependency names were checked against the source imports and package
manifests. These installation instructions have not been run on a Pi.
The targeted build above does not require the optional simulation or
feature-branch visualization packages.

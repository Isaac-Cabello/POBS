# POBS ROS 2 + Gazebo Workspace

A reproducible team development environment for ROS 2 Lyrical Luth, Turtlesim, and Gazebo Jetty.

The project runs ROS 2 and Gazebo in a Docker container based on Ubuntu 26.04. This keeps the team's ROS, Gazebo, compiler, and dependency versions consistent across machines.

## What is included

- ROS 2 Lyrical Luth
- Ubuntu 26.04 inside Docker
- Turtlesim for ROS 2 onboarding
- Gazebo Jetty for simulation
- `pobs_gazebo`, the project Gazebo package
- `2-wheeled_bot`, a simple two-wheeled differential-drive robot
- Windows + WSLg GUI support through `compose.gui-wslg.yaml`

## Important: host ROS installations

You do not need to install ROS 2 or Gazebo directly on Windows or macOS for this project. ROS 2 Lyrical and Gazebo run inside the Docker container.

If you already have ROS 2 installed natively on Windows or macOS, it is fine to keep it installed. It should not break the Docker container. However, use the Docker/WSL workflow in this README for all POBS project work:

- Run project Docker commands from Ubuntu WSL on Windows.
- Run `ros2`, `colcon`, `rosdep`, `gz`, and Gazebo commands inside the project container.
- Do not build or test this repository's Linux ROS packages using a native Windows/macOS ROS installation.
- When reporting an issue, state whether the command ran in PowerShell, a WSL/macOS terminal, or inside the Docker container.

## Team workflow rules

- Git is the shared source of truth.
- Docker images and volumes are local to each team member and are not shared.
- Commit source code, worlds, robot models, launch files, Docker configuration, and documentation.
- Do not commit `build/`, `install/`, `log/`, user credentials, GitHub tokens, SSH keys, or personal configuration files.
- Everyone must finish the Turtlesim onboarding before working on the Gazebo robot.
- The required simulation workflow is headless Gazebo. GUI support is optional and host-specific.

## Repository layout

```text
POBS-ROS2/
├── Dockerfile
├── compose.yaml
├── compose.gui-wslg.yaml         # Windows + WSLg only
├── .dockerignore
├── .gitignore
├── README.md
└── ros2_ws/
    └── src/
        ├── pobs_demo/
        └── pobs_gazebo/
            ├── launch/
            ├── models/
            │   └── 2-wheeled_bot/
            ├── worlds/
            │   └── empty.sdf
            ├── package.xml
            ├── setup.py
            └── setup.cfg
```

# Windows setup

Windows users should use Windows 11, Docker Desktop, WSL 2, Ubuntu WSL, and WSLg.

## 1. Install or update WSL

Open PowerShell as Administrator:

```powershell
wsl --install -d Ubuntu
wsl --update
```

Restart Windows if asked.

Verify Ubuntu uses WSL 2:

```powershell
wsl -l -v
```

Expected shape of the output:

```text
  NAME              STATE           VERSION
* Ubuntu            Running         2
  docker-desktop    Running         2
```

If Ubuntu is version 1, convert it using the exact distro name shown above:

```powershell
wsl --set-version Ubuntu 2
```

## 2. Configure Docker Desktop

Install and start Docker Desktop.

In Docker Desktop:

1. Open **Settings** > **General**.
2. Enable **Use the WSL 2 based engine**.
3. Select **Apply & Restart**.
4. Open **Settings** > **Resources** > **WSL Integration**.
5. Enable integration for your Ubuntu distribution.
6. Select **Apply & Restart**.

## 3. Open Ubuntu WSL

From PowerShell:

```powershell
wsl -d Ubuntu
```

Your prompt should look similar to:

```text
isaac@DESKTOP-NAME:~$
```

All remaining Windows instructions use the Ubuntu WSL terminal unless they explicitly say PowerShell.

Confirm Docker is available:

```bash
docker version
docker ps
```

`docker version` should display both Client and Server information.

If Docker reports permission denied:

1. Confirm Docker Desktop is running.
2. Confirm Docker Desktop WSL Integration is enabled for Ubuntu.
3. Exit Ubuntu.
4. In PowerShell, run:

```powershell
wsl --shutdown
```

5. Reopen Docker Desktop, wait for it to start, then reopen Ubuntu.

# macOS setup

macOS users can use the same Git repository and Docker image for ROS 2 command-line development, Turtlesim topic/service exercises, and headless Gazebo.

## 1. Install Docker and Git

Install Docker Desktop for the correct Mac type:

- Apple Silicon for M-series Macs: M1, M2, M3, M4, etc.
- Intel for Intel-based Macs.

Install Apple command-line tools if Git is not already available:

```bash
xcode-select --install
```

Check the processor architecture:

```bash
uname -m
```

Typical results:

```text
arm64
```

means Apple Silicon, and:

```text
x86_64
```

means Intel.

## 2. Docker limitations on macOS

Do not use the Windows-only GUI file:

```bash
compose.gui-wslg.yaml
```

It requires WSLg paths such as `/mnt/wslg`, which do not exist on macOS.

The required macOS workflow is:

- Dockerized ROS 2 command-line work
- Turtlesim ROS graph/topic/service exercises
- Headless Gazebo
- Git collaboration

Turtlesim/Gazebo GUI in a Linux Docker container on macOS is optional and may require a separate VNC/noVNC or XQuartz-based setup. Do not rely on it for the basic project workflow.

## 3. Apple Silicon note

The ROS image must support ARM64 for native Apple Silicon operation. Start by building normally. If Docker reports an architecture-manifest error, report the full error to the team before adding `platform: linux/amd64`; AMD64 emulation can work but will be slower, especially for Gazebo.

# Clone the project

## Windows / WSL

In Ubuntu WSL, keep the repository in the Linux filesystem when possible:

```bash
mkdir -p ~/projects
cd ~/projects
git clone https://github.com/Isaac-Cabello/POBS.git POBS-ROS2
cd POBS-ROS2
```

## macOS

In Terminal:

```bash
mkdir -p ~/projects
cd ~/projects
git clone https://github.com/Isaac-Cabello/POBS.git POBS-ROS2
cd POBS-ROS2
```

# Build the Docker environment

From the repository root:

```bash
docker compose build
```

The first build downloads the Ubuntu/ROS image and installs ROS development tools, Turtlesim, Gazebo integration packages, and project dependencies. It can take several minutes.

For normal rebuilds after Dockerfile changes:

```bash
docker compose build
```

For a fresh build that ignores cache:

```bash
docker compose build --no-cache
```

# Open the ROS container

For command-line or headless work:

```bash
docker compose run --rm ros
```

Inside the container, the prompt should resemble:

```text
ros@docker-desktop:~/ros2_ws$
```

On macOS, the hostname may be different. That is normal.

## First-time workspace build

Inside the container:

```bash
cd ~/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Normal daily build

After pulling project changes:

```bash
docker compose run --rm ros
```

Then inside the container:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

# Required onboarding: Turtlesim

Everyone must complete these tasks before changing or controlling the Gazebo robot.

Turtlesim teaches ROS 2 nodes, topics, message types, publishers, subscribers, services, and velocity commands. These are the same concepts used later to command a differential-drive robot in Gazebo.

## Windows GUI container command

Windows users with WSLg must run GUI programs from Ubuntu WSL using:

```bash
docker compose -f compose.yaml -f compose.gui-wslg.yaml run --rm ros
```

Do not run this command in PowerShell.

## macOS and headless users

macOS users should still complete the ROS CLI portions of the Turtlesim exercises. Turtlesim GUI is optional on macOS. If no GUI is available, coordinate with a Windows/WSLg or Linux teammate for the visual driving portion, while independently completing topic, message, service, and command exercises.

## Exercise 1: Start Turtlesim

### Windows with WSLg

Terminal 1, in Ubuntu WSL:

```bash
cd ~/projects/POBS-ROS2
docker compose -f compose.yaml -f compose.gui-wslg.yaml run --rm ros
```

Inside the container:

```bash
ros2 run turtlesim turtlesim_node
```

A Turtlesim window should appear on the Windows desktop.

### macOS or a headless environment

Start the node from a container:

```bash
docker compose run --rm ros
ros2 run turtlesim turtlesim_node
```

If the GUI cannot open, leave the node running and complete the CLI exercises from another terminal.

## Exercise 2: Drive the turtle

Open Terminal 2 and start another container.

### Windows with WSLg

```bash
cd ~/projects/POBS-ROS2
docker compose -f compose.yaml -f compose.gui-wslg.yaml run --rm ros
ros2 run turtlesim turtle_teleop_key
```

Click Terminal 2 so it receives keyboard input, then use arrow keys:

- Up: forward
- Down: backward
- Left: turn left
- Right: turn right

### macOS / no GUI

Run the teleoperation node if desired, but you may not see the visual turtle window:

```bash
docker compose run --rm ros
ros2 run turtlesim turtle_teleop_key
```

## Exercise 3: Inspect the ROS graph

Open Terminal 3 and enter a container:

```bash
cd ~/projects/POBS-ROS2
docker compose run --rm ros
```

Inside the container:

```bash
ros2 node list
ros2 topic list
ros2 topic info /turtle1/cmd_vel
ros2 interface show geometry_msgs/msg/Twist
```

Important topics include:

```text
/turtle1/cmd_vel
/turtle1/pose
```

Watch pose updates while the turtle is driven:

```bash
ros2 topic echo /turtle1/pose
```

## Exercise 4: Publish a command

In Terminal 3, publish one `Twist` command:

```bash
ros2 topic pub --once /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

Run a continuous command to draw a circle, then stop it after a few seconds with `Ctrl+C`:

```bash
ros2 topic pub -r 10 /turtle1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
```

## Exercise 5: Call a service

Find the spawn service:

```bash
ros2 service list | grep spawn
```

Spawn a second turtle:

```bash
ros2 service call /spawn turtlesim/srv/Spawn \
  "{x: 7.0, y: 5.0, theta: 0.0, name: 'turtle2'}"
```

Confirm the new turtle has topics:

```bash
ros2 topic list | grep turtle2
```

## Turtlesim completion checklist

Complete all of the following before working on the Gazebo robot:

- [ ] Start `turtlesim_node`
- [ ] Use `turtle_teleop_key` to publish movement commands
- [ ] Identify `/turtle1/cmd_vel`
- [ ] Identify `/turtle1/pose`
- [ ] Run `ros2 node list`
- [ ] Run `ros2 topic list`
- [ ] Inspect `geometry_msgs/msg/Twist`
- [ ] Use `ros2 topic pub` to send a velocity command
- [ ] Spawn `turtle2` with the `/spawn` service
- [ ] Explain: a publisher writes messages to a topic, and subscribers receive those messages

# Gazebo: POBS two-wheeled robot

After completing Turtlesim, use Gazebo to work with `2-wheeled_bot`.

The simulation package contains:

```text
ros2_ws/src/pobs_gazebo/
├── worlds/empty.sdf
└── models/2-wheeled_bot/
    ├── model.config
    └── model.sdf
```

`2-wheeled_bot` is a simple differential-drive robot with a chassis, two driven wheels, a rear caster, collision geometry, physics properties, and a Gazebo Differential Drive system plugin.

The conceptual mapping is:

```text
Turtlesim:
teleop node -> /turtle1/cmd_vel -> turtle simulator

Gazebo:
command publisher -> /model/2-wheeled_bot/cmd_vel -> DiffDrive plugin -> robot wheels
```

## Run Gazebo headlessly: required workflow

This works on Windows, macOS, Linux, and CI.

From the repository root:

```bash
docker compose run --rm ros
```

Inside the container:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src/pobs_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

gz sim -s -r "$HOME/ros2_ws/src/pobs_gazebo/worlds/empty.sdf"
```

- `-s` runs only the Gazebo server, without a GUI.
- `-r` starts the simulation immediately.
- Stop the simulator with `Ctrl+C`.

## Run Gazebo GUI: Windows + WSLg

This is currently the supported GUI path for the team.

From Ubuntu WSL at the repository root:

```bash
docker compose -f compose.yaml -f compose.gui-wslg.yaml run --rm ros
```

Inside the container:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src/pobs_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

gz sim "$HOME/ros2_ws/src/pobs_gazebo/worlds/empty.sdf"
```

A Gazebo window should open on the Windows desktop.

## macOS Gazebo GUI

macOS users should use headless Gazebo as the required workflow. Do not use `compose.gui-wslg.yaml` on macOS.

A future, separately tested macOS GUI solution may use VNC/noVNC or another remote-display method. Until that is documented, do not block project work on Gazebo GUI availability on a Mac.

## Drive `2-wheeled_bot`

Keep Gazebo running in one terminal/container.

Open a second terminal at the repository root and enter another container:

```bash
docker compose run --rm ros
```

Inside the second container, command the robot directly through Gazebo Transport.

Drive forward:

```bash
gz topic -t /model/2-wheeled_bot/cmd_vel \
  -m gz.msgs.Twist \
  -p 'linear: {x: 0.3}, angular: {z: 0.0}'
```

Turn in place:

```bash
gz topic -t /model/2-wheeled_bot/cmd_vel \
  -m gz.msgs.Twist \
  -p 'linear: {x: 0.0}, angular: {z: 0.8}'
```

Stop:

```bash
gz topic -t /model/2-wheeled_bot/cmd_vel \
  -m gz.msgs.Twist \
  -p 'linear: {x: 0.0}, angular: {z: 0.0}'
```

# Current status and limitation

Working:

- Dockerized ROS 2 Lyrical development environment
- Turtlesim and ROS 2 command-line exercises
- Windows GUI forwarding through WSLg
- Gazebo Jetty GUI on Windows/WSLg
- Gazebo Jetty headless simulation
- `pobs_gazebo` world/model package
- Direct Gazebo Transport velocity commands for `2-wheeled_bot`

Current limitation:

- `ros2 launch ros_gz_sim ...` currently fails in this image with a ROS 2 Lyrical binary type-support symbol mismatch involving `simulation_interfaces` and `unique_identifier_msgs`.
- The failure does not prevent Turtlesim, direct Gazebo, world/model development, or direct Gazebo Transport control.
- Do not use `ros2 launch ros_gz_sim ...` as the team-standard launcher until the repository documents a verified fix.

# Git workflow

Before beginning work:

```bash
cd ~/projects/POBS-ROS2
git pull
docker compose build
```

Check what changed:

```bash
git status
```

Commit only your intended changes:

```bash
git add <files-you-changed>
git commit -m "Describe your change"
git push
```

Examples:

```bash
git add ros2_ws/src/pobs_gazebo/worlds/empty.sdf
git commit -m "Add obstacle to test world"
git push
```

```bash
git add ros2_ws/src/pobs_gazebo/models/2-wheeled_bot/model.sdf
git commit -m "Tune wheel friction"
git push
```

Do not commit generated files:

```text
ros2_ws/build/
ros2_ws/install/
ros2_ws/log/
```

Do not commit passwords, API keys, GitHub tokens, SSH keys, or personal `.env` files.

# Docker safety

Do not run this command unless you intentionally want to remove your local Docker build/install/log volumes:

```bash
docker compose down -v
```

Normal commands such as these are safe:

```bash
docker compose build
docker compose run --rm ros
docker compose down
```

# Troubleshooting

## Docker permission denied in Ubuntu WSL

Confirm Docker Desktop is open and WSL Integration is enabled for Ubuntu. Then from PowerShell:

```powershell
wsl --shutdown
```

Reopen Ubuntu and test:

```bash
docker ps
```

## GUI cannot connect to display on Windows

Use the GUI overlay from Ubuntu WSL, not PowerShell:

```bash
docker compose -f compose.yaml -f compose.gui-wslg.yaml run --rm ros
```

In Ubuntu WSL, verify:

```bash
echo "$DISPLAY"
ls /mnt/wslg
```

If needed, restart Docker Desktop and run `wsl --shutdown` from PowerShell.

## Gazebo cannot find `2-wheeled_bot`

Set the resource path before launching the world:

```bash
export GZ_SIM_RESOURCE_PATH="$HOME/ros2_ws/src/pobs_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"
```

## ROS does not find a changed package

Rebuild and source the workspace:

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Mac image architecture error

Run:

```bash
uname -m
docker compose build
```

Copy the complete Docker error into a GitHub issue. Do not silently add `platform: linux/amd64` without telling the team, because emulation changes performance and may affect Gazebo behavior.

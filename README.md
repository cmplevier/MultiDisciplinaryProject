# MDP 2026 — FloraNova Greenhouse Robot

TU Delft RO47007 Multidisciplinary Project. MIRTE Master V2, ROS2 Humble.

## Setup

```bash
git clone https://gitlab.tudelft.nl/cor/ro47007/2026/group_25/mdp2026.git
cd mdp2026
```
or (if using ssh key)

```bash
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_25/mdp2026.git
cd mdp2026
```


Open in VS Code → **Reopen in Container** (requires Docker + Dev Containers extension).  
The container will automatically pull all MIRTE dependencies via `vcs import` and `rosdep`. Then build:

```bash
colcon build --symlink-install
source install/setup.bash
```

Or natively on Ubuntu 22:
```bash
source /opt/ros/humble/setup.bash
mkdir src
vcs import src/ < sources.repos
vcs import src/ < src/mirte-gazebo/sources.repos
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If you get issues during the colcon build of mirte_telemetrix_cppsaying:

```bash 
Failed   <<< mirte_telemetrix_cppsaying 
```

It is because `libs/tmx-cpp` is missing its `CMakeLists.txt` as `tmx-cpp` is a submodule. To Fix:

```bash
cd ~/mdp2026/src/mirte-ros-packages
git submodule update --init --recursive
```

Then go bach to `~/mdp2026` and build again.

## Repo Structure

```
src/
├── mirte-ros-packages/   # MIRTE drivers, URDF, teleop (vcs import)
├── mirte-gazebo/         # Gazebo worlds + robot spawn (vcs import)
├── mdp_bringup/          # Team launch files (sim + real)
├── mdp_gazebo/           # Custom world extensions
├── mdp_plant_monitor/    # Plant health detection + digital twin feed
└── mdp_slm_tts/          # SLM + TTS robot personality
```

## Simulation

Launch Gazebo with MIRTE Master (run twice on first use — first run caches models):

```bash
ros2 launch mirte_gazebo gazebo_mirte_master_empty.launch.xml
```

Keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

## Environment

- `ROS_DOMAIN_ID=42` — set in devcontainer, set manually if running natively
- Offboard laptop and OrangePi must share the same domain ID over LAN
- Run `xhost +local:docker` on host before launching Gazebo in container

# MDP 2026 — FloraNova Greenhouse Robot

TU Delft RO47007 Multidisciplinary Project. MIRTE Master V2, ROS2 Humble.

## Setup

Make a workspace folder and clone the repo inside the /src folder:

```bash
mkdir -p mdp_ws/src
cd mdp_ws/src
git clone https://gitlab.tudelft.nl/cor/ro47007/2026/group_25/mdp2026.git
```

or (if using ssh key)

```bash
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_25/mdp2026.git
```

From the /src folder, clone the source repos:

```bash
cd src
vcs import < mdp-packages/sources.repos
```

Return to workspace root:

```bash
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
cd ~mdp_ws/src/mirte-ros-packages
git submodule update --init --recursive
```

Then go bach to `~/mdp2026` and build again.

## Repo Structure

```
mdp_bringup/          # Team launch files (sim + real)
mdp_gazebo/           # Custom world extensions
mdp_plant_monitor/    # Plant health detection + digital twin feed
mdp_slm_tts/          # SLM + TTS robot personality
```

## Simulation And Navigation

Launch Gazebo with MIRTE Master in the custom greenhouse world. Run this from the workspace root in one terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mdp_gazebo greenhouse_world.launch.xml rviz:=false
```

The `rviz:=false` argument keeps Gazebo from opening its own RViz window, because the navigation launch below opens the combined localization/navigation RViz config.

In a second terminal, launch localization, Nav2, and RViz together:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mdp_navigation nav2_with_localization.launch.py
```

This uses the default map and parameter files:

```text
map: mdp_localization/maps/asym_map.yaml
localization params: mdp_localization/config/amcl_params.yaml
navigation params: mdp_navigation/config/nav2_params.yaml
rviz config: mdp_navigation/rviz/combined.rviz
```

To use another map or RViz config:

```bash
ros2 launch mdp_navigation nav2_with_localization.launch.py \
  map:=/absolute/path/to/map.yaml \
  rviz_config_file:=/absolute/path/to/config.rviz
```

Keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

## Environment

- `ROS_DOMAIN_ID=0` — set in devcontainer, set manually if running natively
- Offboard laptop and OrangePi must share the same domain ID over LAN
- Run `xhost +local:docker` on host before launching Gazebo in container

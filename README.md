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

### Simulation

Launch Gazebo with MIRTE Master in the custom greenhouse world. Run this from the workspace root in one terminal:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mdp_gazebo greenhouse_world.launch.xml rviz:=false
```

The `rviz:=false` argument keeps Gazebo from opening its own RViz window, because the navigation launch below opens the combined localization/navigation RViz config.

In a second terminal, launch localization, Nav2, and RViz in simulation mode:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mdp_navigation nav2_with_localization_and_rviz.launch.py use_sim_time:=true
```

Simulation mode uses:

```text
clock: simulation time
odom topic: /odom
Nav2 cmd_vel output: /mirte_base_controller/cmd_vel_unstamped
```

Simulation keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

### Real Robot

On the real robot, first start the MIRTE hardware bringup:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mirte_bringup minimal_master.launch.py
```

In another terminal, launch localization, Nav2, and RViz with the default real-robot settings:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch mdp_navigation nav2_with_localization_and_rviz.launch.py
```

Real robot mode uses:

```text
clock: wall time
odom topic: /mirte_base_controller/odom
Nav2 cmd_vel output: /mirte_base_controller/cmd_vel
```

Real robot keyboard teleop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/mirte_base_controller/cmd_vel
```

Before starting navigation goals, these checks must pass:

```bash
ros2 topic hz /mirte_base_controller/odom
ros2 run tf2_ros tf2_echo odom base_link
ros2 control list_controllers
```

The `tf2_echo` command must show a live transform from `odom` to `base_link`. The controllers list should show `mirte_base_controller` active. If `odom -> base_link` is missing, Nav2 can make a global plan but the local costmap and velocity controller will not work.

### Shared Options

Both simulation and real robot launches use the default map and parameter files:

```text
map: mdp_localization/maps/asym_map.yaml
localization params: mdp_localization/config/amcl_params.yaml
navigation params: mdp_navigation/config/nav2_params.yaml
rviz config: mdp_navigation/rviz/combined.rviz
```

To use another map or RViz config on the real robot:

```bash
ros2 launch mdp_navigation nav2_with_localization_and_rviz.launch.py \
  map:=/absolute/path/to/map.yaml \
  rviz_config_file:=/absolute/path/to/config.rviz
```

In simulation, keep `use_sim_time:=true` in the same command:

```bash
ros2 launch mdp_navigation nav2_with_localization_and_rviz.launch.py \
  use_sim_time:=true \
  map:=/absolute/path/to/map.yaml \
  rviz_config_file:=/absolute/path/to/config.rviz
```

The `map` argument must point to a real `.yaml` file, not the placeholder path above. If the map does not appear in RViz, check that localization loaded it and that the robot TF chain exists:

```bash
ros2 lifecycle get /map_server
ros2 topic echo --once /map
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo map odom
```

`/map` comes from `map_server`. The `map -> odom` transform and `/amcl_pose` come from AMCL, but AMCL needs live laser scans and an existing `odom -> base_link` transform first. If `odom -> base_link` is missing, start the Gazebo or real-robot bringup and make sure `mirte_base_controller` is active before sending Nav2 goals.

For custom setups, the important launch arguments are:

```bash
use_sim_time:=true|false
odom_topic:=/odom|/mirte_base_controller/odom
cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped|/mirte_base_controller/cmd_vel
```

## Environment

- `ROS_DOMAIN_ID=0` — set in devcontainer, set manually if running natively
- Offboard laptop and OrangePi must share the same domain ID over LAN
- Run `xhost +local:docker` on host before launching Gazebo in container

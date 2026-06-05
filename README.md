# MDP 2026 — FloraNova Greenhouse Robot

TU Delft RO47007 Multidisciplinary Project. MIRTE Master V2, ROS2 Humble.

## Setup

Make a workspace folder and clone the repo inside the /src folder:

```bash
mkdir -p mdp_ws/src
cd mdp_ws/src
git clone https://gitlab.tudelft.nl/cor/ro47007/2026/group_25/mdp-packages.git
```

or (if using ssh key)

```bash
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_25/mdp-packages.git
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

## Row Plan Authoring And Mission Execution

The goal of this pipeline is to scan trays from a discrete plan. Each
tray has four waypoints: the robot navigates to `A`, strafes from `A` to
`B`, navigates from `B` to `C`, then strafes from `C` to `D`.

The system is intentionally modular so each part can be tested
separately before the robot moves:

```text
row_plan_builder_node
  Used during authoring only.
  Receives RViz arrows or JSON commands.
  Writes ~/mdp_ws/generated_row_plan.json.
  Publishes RViz markers so you can visually check the plan.

row_plan_validator_node
  Used after authoring.
  Reads the generated JSON file.
  Checks that trays, IDs, and A/B/C/D poses are valid.

mdp_navigation stack
  Starts simulation, localization, map server, Nav2, and RViz.
  Provides /amcl_pose and the /navigate_to_pose action server.
  Publishes Nav2 velocity commands for the approach motion.

mainloop_node
  Used during execution.
  Reads ~/mdp_ws/generated_row_plan.json.
  Sends each tray segment start pose to Nav2.
  After Nav2 succeeds, publishes direct strafe velocity commands until
  the segment end pose is reached.
```

The data flow is:

```text
RViz arrows
  -> row_plan_builder_node
  -> generated_row_plan.json
  -> row_plan_validator_node
  -> mainloop_node
  -> Nav2 approach + direct strafe
```

The launch flow is split into two phases so pose creation and robot
execution can be debugged separately:

```text
1. Author the row plan.
   Create and inspect generated_row_plan.json. The robot does not move.

2. Execute the finished plan.
   Start navigation, start the executor, then enable autonomy.
```

### 0. Build

Run this after changing any package code or launch files:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mdp_mainloop mdp_navigation
source install/setup.bash
```

### 1. Author The Row Plan

This creates the JSON file only. It starts a map server, RViz, and the
row-plan builder. It does not start Nav2 navigation or the mission
executor, so the robot will not move.

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_mainloop row_plan_authoring.launch.py \
  clear_plan:=true \
  plan_path:=~/mdp_ws/generated_row_plan.json
```

The generated file is written here:

```text
~/mdp_ws/generated_row_plan.json
```

The builder defaults to tray capture mode. In RViz, use the two
row-plan pose tools:

```text
Set Row Approach Pose -> /row_plan/approach_pose
Set Row Goal Pose     -> /row_plan/scan_end_pose
```

The toolbar may show both as `2D Goal Pose`; check the Tool Properties
panel if the labels are ambiguous (the left one is the approach one, while the right one is the goal strafing pose).

For each tray:

```text
1. Publish a tray ID.
2. Set the approach/start arrow for A.
3. Set the goal/end arrow for B.
4. Set the approach/start arrow for C.
5. Set the goal/end arrow for D.
```

Example:

```bash
ros2 topic pub --once /row_plan/tray_id std_msgs/String "{data: tray_1}"
```

Then click-drag `Set Row Approach Pose`, then click-drag
`Set Row Goal Pose` for `A -> B`. Repeat the two clicks for `C -> D`.
Then publish `tray_2`, `tray_3`, etc.

The builder shows:

```text
blue arrow  = segment start
green arrow = segment end
line        = strafe segment
```

To check the file before running the robot:

```bash
cat ~/mdp_ws/generated_row_plan.json
python3 -m json.tool ~/mdp_ws/generated_row_plan.json

ros2 run mdp_mainloop row_plan_validator_node \
  --ros-args -p plan_path:=~/mdp_ws/generated_row_plan.json
```

The validator reports the row order and any missing/invalid poses.

### 2. Execute The Finished Plan

Close the authoring launch with `Ctrl+C`. Before launching navigation,
this command should not show `mdp_row_plan_builder_node` or
`lifecycle_manager_row_plan_authoring`. If it does, the authoring launch
is still running.

```bash
ros2 node list | grep -E "map_server|amcl|lifecycle|row_plan"
```

Terminal 1: start simulation, localization, Nav2, and RViz. Nav2 will
publish approach-motion velocity commands to
`/mirte_base_controller/cmd_vel_unstamped`.

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_navigation sim_nav_loc_rviz.launch.py \
  use_sim_time:=true \
  cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped
```

In RViz, set the robot's initial pose with `2D Pose Estimate` if AMCL
does not already know where the robot is.

Terminal 2: start the mission executor. It reads
`generated_row_plan.json`, sends Nav2 goals for each segment start, and
publishes strafe velocity commands to the same velocity topic.

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_mainloop mainloop.launch.py \
  use_sim_time:=true \
  clear_history:=true \
  cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped \
  strafe_block_unknown_costmap:=true \
  strafe_block_timeout_sec:=8.0 \
  blocked_tray_retry_delay_sec:=60.0 \
  plan_path:=~/mdp_ws/generated_row_plan.json
```

This launch starts:

```text
mdp_mainloop_node  # executes tray NAV + STRAFE tasks
```

The executor follows the tray order in the JSON file. During strafing,
the local costmap filters the strafe command. Occupied cells, and unknown
cells when `strafe_block_unknown_costmap:=true`, make the robot publish a
zero velocity and wait. If the strafe stays blocked longer than
`strafe_block_timeout_sec`, the whole tray is skipped, the next plan
choice is a random unblocked tray, and then the executor continues from
that point in JSON order. The skipped tray remains blocked until
`blocked_tray_retry_delay_sec` expires.

Mission history is now a run log, not a permanent skip list. Completed
segments are skipped only inside the current mission pass. Restarting the
executor, or launching with `loop_mission:=true`, lets finished trays be
visited again while preserving their `completed_count` in the history
file. If a tray is blocked and all other unfinished work is done, the
executor can start another pass over the unblocked trays while waiting
for the blocked tray cooldown.

Terminal 3: check that the executor has a robot pose and is waiting for
autonomy.

```bash
source /opt/ros/humble/setup.bash
source ~/mdp_ws/install/setup.bash

ros2 topic echo --once /amcl_pose
ros2 topic echo --once /mainloop/status
```

If `/amcl_pose` does not print, use `2D Pose Estimate` in RViz and check
again.

Enable autonomy by publishing the enable message for a few seconds. This
makes sure the executor receives it.

```bash
ros2 topic pub -r 2 /autonomous_enabled std_msgs/Bool "{data: true}"
```

Leave it running until `/mainloop/status` contains:

```text
"autonomous_enabled": true
```

Then stop the publisher with `Ctrl+C`. The mission should move from
`TASK_READY` to `NAVIGATING_TO_APPROACH`, then later to `STRAFING_ROW`.

Useful checks while running:

```bash
ros2 topic echo /mission_dashboard
ros2 topic echo /mainloop/status
ros2 topic echo /mainloop/task_result
ros2 topic echo /mirte_base_controller/cmd_vel_unstamped
```

If the dashboard says `EXECUTOR: TASK_READY` and no velocity appears,
check:

```bash
ros2 topic echo --once /mainloop/status
ros2 topic echo --once /amcl_pose
```

Common causes are:

```text
autonomous_enabled is false in /mainloop/status
/amcl_pose is missing because the initial pose was not set
Nav2 is not active yet
generated_row_plan.json is missing or invalid
```

### Optional Dynamic Updates

To add or replace one tray dynamically:

```bash
ros2 topic pub --once /row_plan/set_tray std_msgs/String \
  "{data: '{\"id\": \"tray_3\", \"waypoints\": {\"A\": [1.0, 0.0, 1.57], \"B\": [1.0, 1.2, 1.57], \"C\": [1.4, 1.2, -1.57], \"D\": [1.4, 0.0, -1.57]}}'}"
```

The builder writes the generated JSON file and publishes the full active
plan on `/planner/row_plan` for visualization/debugging. For execution,
restart `mainloop.launch.py` or point it at the updated JSON file.

The executor reports status on `/mainloop/status` and task results on
`/mainloop/task_result`. Autonomy is still gated by `/autonomous_enabled`.

## Environment

- `ROS_DOMAIN_ID=0` — set in devcontainer, set manually if running natively
- Offboard laptop and OrangePi must share the same domain ID over LAN
- Run `xhost +local:docker` on host before launching Gazebo in container

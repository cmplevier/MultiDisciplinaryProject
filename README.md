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

The goal of this pipeline is to scan greenhouse rows from a discrete
plan. For each row, the robot first navigates to an `approach_pose` using
Nav2, then strafes directly to a `scan_end_pose` while scanning.

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
  Checks that rows, IDs, approach poses, and scan-end poses are valid.

mdp_navigation stack
  Starts simulation, localization, map server, Nav2, and RViz.
  Provides /amcl_pose and the /navigate_to_pose action server.
  Publishes Nav2 velocity commands for the approach motion.

high_level_planner_node
  Used during execution.
  Reads ~/mdp_ws/generated_row_plan.json.
  Chooses the next row, normally in JSON order.
  Sends one row task at a time to mainloop_node.

mainloop_node
  Used during execution.
  Receives row tasks from the planner.
  Sends approach_pose to Nav2.
  After Nav2 succeeds, publishes direct strafe velocity commands until
  scan_end_pose is reached.
```

The data flow is:

```text
RViz arrows
  -> row_plan_builder_node
  -> generated_row_plan.json
  -> row_plan_validator_node
  -> high_level_planner_node
  -> mainloop_node
  -> Nav2 approach + direct strafe
```

The launch flow is split into two phases so pose creation and robot
execution can be debugged separately:

```text
1. Author the row plan.
   Create and inspect generated_row_plan.json. The robot does not move.

2. Execute the finished plan.
   Start navigation, start the planner/executor, then enable autonomy.
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

In RViz, use the two row-plan pose tools:

```text
Set Row Approach Pose -> /row_plan/approach_pose
Set Row Goal Pose     -> /row_plan/scan_end_pose
```

The toolbar may show both as `2D Goal Pose`; check the Tool Properties
panel if the labels are ambiguous (the left one is the approach one, while the right one is the goal strafing pose).

For each row:

```text
1. Publish a row ID.
2. Set the approach arrow where Nav2 should drive first.
3. Set the goal / scan-end arrow where strafing should finish.
```

Example:

```bash
ros2 topic pub --once /row_plan/row_id std_msgs/String "{data: row_1}"
```

Then click-drag `Set Row Approach Pose`, then click-drag
`Set Row Goal Pose`. Repeat with `row_2`, `row_3`, etc.

The builder shows:

```text
blue arrow  = APPROACH
green arrow = GOAL / SCAN END
line        = connection between them
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

Terminal 2: start only the mission planner and executor. The planner
reads `generated_row_plan.json`; the executor sends Nav2 goals and later
publishes strafe velocity commands to the same velocity topic.

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_mainloop mainloop.launch.py \
  use_sim_time:=true \
  clear_history:=true \
  cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped \
  generated_row_plan_path:=~/mdp_ws/generated_row_plan.json
```

This launch starts:

```text
mdp_high_level_planner_node  # reads generated_row_plan.json
mdp_mainloop_node            # executes NAV + STRAFE tasks
```

If no row scores are published, the planner follows the order in the
JSON file: first row, second row, third row, and so on.

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
makes sure both the planner and executor receive it.

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
ros2 topic echo /planner/status
ros2 topic echo /planner/discrete_state
ros2 topic echo /planner/next_task
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

To add or replace one row dynamically:

```bash
ros2 topic pub --once /row_plan/set_row std_msgs/String \
  "{data: '{\"id\": \"row_c\", \"approach_pose\": [1.0, 0.0, 1.57], \"scan_end_pose\": [1.0, 1.2, 1.57]}'}"
```

The builder writes the generated JSON file and publishes the full active
plan on `/planner/row_plan`. The planner reloads that topic while
running.

Planner input can be published as JSON on `/planner/row_scores`:

```bash
ros2 topic pub --once /planner/row_scores std_msgs/String \
  "{data: '{\"row_scores\": {\"row_a\": 0.1, \"row_b\": 0.9}}'}"
```

The planner will choose the highest-scored available row. To force a
specific row:

```bash
ros2 topic pub --once /planner/row_scores std_msgs/String \
  "{data: '{\"selected_row\": \"row_b\", \"force_rescan\": true}'}"
```

The executor reports status on `/mainloop/status` and task results on
`/mainloop/task_result`. Autonomy is still gated by `/autonomous_enabled`.

## Environment

- `ROS_DOMAIN_ID=0` — set in devcontainer, set manually if running natively
- Offboard laptop and OrangePi must share the same domain ID over LAN
- Run `xhost +local:docker` on host before launching Gazebo in container

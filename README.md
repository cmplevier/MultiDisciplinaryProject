## Demo Videos

### IRL Demo
<video src="Demo_videos/IRL_DEMO.mp4" controls width="100%"></video>

### Navigation Demo
<video src="Demo_videos/NAV2_DEMO.mp4" controls width="100%"></video>

---

# MDP 2026 - FloraNova Greenhouse Robot

TU Delft RO47007 Multidisciplinary Project. MIRTE Master V2, ROS 2 Humble.

This README is written as an operator guide. The commands are meant to be
copied into terminals from the workspace root, `~/mdp_ws`.

## Quick Rules

- Use ROS 2 Humble.
- Open a new terminal for each long-running launch command.
- In every new terminal, source ROS and the workspace first.
- Use the automatic tray waypoint labelling workflow:
  `auto_tray_waypoint_authoring.launch.py`.
- Do not use the old manual `row_plan_authoring.launch.py` workflow unless you
  are intentionally debugging the old builder.

## Repository Layout

```text
mdp_bringup/        Team bringup helpers, including the real-robot twist_mux
mdp_gazebo/         Gazebo greenhouse world and simulated MIRTE launch
mdp_localization/   AMCL config and saved maps
mdp_navigation/     Nav2, localization, simulation-navigation launch files
mdp_mainloop/       Automatic tray waypoint labelling and mission executor
mdp_perception/     Perception node and model files
mdp_slam/           SLAM toolbox mapping launch and config
mdp_teleop/         Joystick and keyboard teleop helpers
```

## First-Time Setup

Create the workspace and clone this repository:

```bash
mkdir -p ~/mdp_ws/src
cd ~/mdp_ws/src
git clone https://gitlab.tudelft.nl/cor/ro47007/2026/group_25/mdp-packages.git
```

SSH clone alternative:

```bash
mkdir -p ~/mdp_ws/src
cd ~/mdp_ws/src
git clone git@gitlab.tudelft.nl:cor/ro47007/2026/group_25/mdp-packages.git
```

Import the external repositories listed in `sources.repos`:

```bash
cd ~/mdp_ws/src
vcs import < mdp-packages/sources.repos
```

Install ROS dependencies and build:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

If `mirte_telemetrix_cpp` fails because `libs/tmx-cpp` is missing a
`CMakeLists.txt`, initialize the MIRTE submodules and build again:

```bash
cd ~/mdp_ws/src/mirte-ros-packages
git submodule update --init --recursive

cd ~/mdp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Build After Code Changes

Use this after changing package code, launch files, config files, or this repo:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

For a faster build when only the MDP packages changed:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install \
  --packages-select \
  mdp_bringup \
  mdp_gazebo \
  mdp_localization \
  mdp_navigation \
  mdp_mainloop \
  mdp_perception \
  mdp_slam \
  mdp_teleop
source install/setup.bash
```

## Terminal Setup

Run this at the top of every new terminal:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Optional, if the robot/laptop network uses a specific ROS domain:

```bash
export ROS_DOMAIN_ID=0
```

## Simulation

### Start Simulation, Localization, Nav2, And RViz

Terminal 1:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_navigation sim_nav_loc_rviz.launch.py \
  use_sim_time:=true \
  cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped
```

This one command starts Gazebo, the greenhouse world, the MIRTE robot,
localization, Nav2, and RViz.

Parameter meanings:

```text
use_sim_time
  true means all nodes use the Gazebo /clock.

cmd_vel_topic
  Velocity topic used by Nav2 in simulation.
  Use /mirte_base_controller/cmd_vel_unstamped for the simulated MIRTE base.
```

Simulation uses the greenhouse/asymmetric map:

```text
mdp_localization/maps/asym_map.yaml
```

Do not use the final real-robot map in simulation. The simulated world is the
greenhouse configuration, so the navigation map must match that environment.

### Simulation Teleop

Keyboard teleop directly to the simulated base:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r /cmd_vel:=/mirte_base_controller/cmd_vel_unstamped
```

### Simulation Checks

Use these in another terminal:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic hz /mirte_base_controller/odom
```

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run tf2_ros tf2_echo odom base_link
```

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
ros2 action list | grep navigate_to_pose
```

Before sending goals, RViz should show the map, laser scan, robot model,
and a reasonable robot pose. If AMCL does not know the pose, use `2D Pose
Estimate` in RViz.

## Real Robot

### Start MIRTE Hardware

Terminal 1 on the robot:


```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_bringup twist_mux.launch.py
```

The MDP `twist_mux` listens to:

```text
cmd_vel_joy    joystick teleop, highest priority
cmd_vel_key    keyboard teleop
cmd_vel_stop   stop command
cmd_vel_nav    Nav2 and mission executor
cmd_vel_idle   low-priority idle command
```

It outputs to:

```text
/mirte_base_controller/cmd_vel
```

### Start Real-Robot Localization, Nav2, And RViz

Terminal 3, usually on the laptop that displays RViz:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_navigation nav2_with_localization_and_rviz.launch.py \
  use_sim_time:=false \
  cmd_vel_topic:=/cmd_vel_nav
```

Parameter meanings:

```text
use_sim_time
  false means all nodes use wall-clock time, which is required on the real robot.

cmd_vel_topic
  Velocity topic used by Nav2.
  Use /cmd_vel_nav when mdp_bringup twist_mux is running.
```

To use another map on the real robot:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_navigation nav2_with_localization_and_rviz.launch.py \
  use_sim_time:=false \
  map:=/absolute/path/to/map.yaml \
  cmd_vel_topic:=/cmd_vel_nav
```

Extra parameter:

```text
map
  Absolute path to the map YAML file loaded by the map server and AMCL.
```

### Real-Robot Teleop

Keyboard teleop through the MDP `twist_mux`:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r /cmd_vel:=/cmd_vel_key
```

Joystick teleop through the MDP `twist_mux`:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_teleop teleop_custom.launch.py \
  cmd_vel_remap:=/cmd_vel_joy \
  joy_config:=custom_ps5.yaml
```

Parameter meanings:

```text
cmd_vel_remap
  Topic where the joystick teleop node publishes velocity commands.
  Use /cmd_vel_joy when mdp_bringup twist_mux is running.

joy_config
  Joystick configuration file from mdp_teleop/config.
  Available configs include custom_ps5.yaml, custom_u22.yaml, and ps4.yaml.
```

### Real-Robot Checks

Run these before starting autonomous motion:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic hz /mirte_base_controller/odom
```

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run tf2_ros tf2_echo odom base_link
```

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
```

Expected:

```text
/mirte_base_controller/odom publishes continuously
odom -> base_link transform is live
mirte_base_controller is active
```

If `odom -> base_link` is missing, Nav2 may create a global path but the
local costmap and controller will not drive correctly.

## Automatic Tray Waypoint Labelling

This is the waypoint authoring workflow to use for the project. It creates
the JSON file that the mission executor reads later.

The automatic labeller reads the occupancy grid map, waits for RViz
`Publish Point` clicks, finds the occupied tray blob under each click,
fits the tray direction, and writes waypoints `A`, `B`, `C`, and `D`.

For each tray:

```text
A -> B is the first lateral scan/strafe segment.
C -> D is the second lateral scan/strafe segment.
```

The generated yaw faces the tray by default, so moving from `A` to `B`
and from `C` to `D` is lateral motion in the robot frame.

### Start The Automatic Labeller

For the real robot, start the labeller with wall-clock time:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_mainloop auto_tray_waypoint_authoring.launch.py \
  use_sim_time:=false \
  map:=/absolute/path/to/real_robot_map.yaml \
  clear_plan:=true \
  plan_path:=~/mdp_ws/generated_row_plan.json \
  longitudinal_margin_m:=0.20 \
  lateral_offset_m:=0.35 \
  click_search_radius_m:=0.25 \
  occupied_threshold:=65
```

Parameter meanings:

```text
use_sim_time
  true in Gazebo, false on the real robot.

map
  Map YAML file used as the RViz background and occupancy grid.
  Use the same map that you will use later for real-robot navigation.

clear_plan
  true starts from an empty JSON file.
  false keeps existing trays and replaces only trays with matching IDs.

plan_path
  JSON file that will be created or updated.
  The mission executor reads this same file later.

longitudinal_margin_m
  Extra distance before and after the occupied tray ends.
  Increase it when the scan should start earlier or finish later.

lateral_offset_m
  Robot-center distance from each side of the tray.
  Increase it if the robot is too close to the tray.

click_search_radius_m
  Search radius around the RViz click if the click misses the occupied cell.
  Increase it if clicks near the tray are ignored.

occupied_threshold
  Occupancy-grid value treated as an obstacle.
  The default 65 works for normal black occupied map regions.
```

### Label Trays In RViz

For each tray, publish the tray ID, then click the black occupied tray
region in RViz using the `Publish Point` tool. The tray ID message applies
to the next click only, so publish the ID immediately before clicking that
tray.

The generated JSON has this shape:

```text
tray_1 -> waypoints A, B, C, D
tray_2 -> waypoints A, B, C, D
tray_3 -> waypoints A, B, C, D
```

If you publish an ID that already exists, the next click replaces that
tray's waypoints. If you do not publish an ID, the node creates the next
available automatic name such as `tray_1` or `tray_2`; for the final mission,
publish explicit IDs so the tray order is clear.

Tray 1:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /row_plan/tray_id std_msgs/String "{data: tray_1}"
```

Then click near the center of tray 1 in RViz with `Publish Point`.

Tray 2:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /row_plan/tray_id std_msgs/String "{data: tray_2}"
```

Then click near the center of tray 2.

Continue with `tray_3`, `tray_4`, and so on.

The labeller publishes:

```text
/planner/row_plan              full generated JSON plan
/row_plan/auto_status          generator status as JSON text
/row_plan/auto_tray_markers    RViz rectangles, arrows, and waypoint labels
```

Useful checks while authoring:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /row_plan/auto_status
```

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 -m json.tool ~/mdp_ws/generated_row_plan.json
```

## Mission Execution On The Real Robot

Follow these steps in order.

### 0. Start and Connect to MIRTE Hardware

This is required before `twist_mux`, teleop, Nav2, or the mainloop can move
the robot.


### 1. Create The JSON Waypoint File

Start the automatic labeller:

Terminal 1:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_mainloop auto_tray_waypoint_authoring.launch.py \
  use_sim_time:=false \
  map:=/absolute/path/to/real_robot_map.yaml \
  clear_plan:=true \
  plan_path:=~/mdp_ws/generated_row_plan.json \
  longitudinal_margin_m:=0.20 \
  lateral_offset_m:=0.35 \
  click_search_radius_m:=0.25 \
  occupied_threshold:=65
```

In RViz, use `Publish Point` to click each black tray region on the map.
Assign the tray ID immediately before the click.

For tray 1:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /row_plan/tray_id std_msgs/String "{data: tray_1}"
```

Then click tray 1 in RViz.

For tray 2:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /row_plan/tray_id std_msgs/String "{data: tray_2}"
```

Then click tray 2 in RViz. Continue with `tray_3`, `tray_4`, and so on.


### 2. Launch The Twist Mux

Terminal 2 on the robot:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_bringup twist_mux.launch.py
```

The MDP `twist_mux` lets teleop and navigation share the real robot velocity
output. Nav2 and the mainloop must publish to `/cmd_vel_nav` when this mux is
running.

### 3. Launch Teleop

Keyboard teleop through `twist_mux`:

Terminal 3:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args \
  -r /cmd_vel:=/cmd_vel_key
```

Joystick teleop alternative:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_teleop teleop_custom.launch.py \
  cmd_vel_remap:=/cmd_vel_joy \
  joy_config:=custom_ps5.yaml
```

Use teleop to verify that the robot can move. Stop teleop commands before
enabling autonomy.

### 4. Launch Navigation With RViz

Terminal 4:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_navigation nav2_with_localization_and_rviz.launch.py \
  use_sim_time:=false \
  cmd_vel_topic:=/cmd_vel_nav \
  map:=/absolute/path/to/real_robot_map.yaml
```

In RViz, set the initial pose with `2D Pose Estimate` if needed. Before
continuing, `/amcl_pose` must publish and Nav2 must have the
`navigate_to_pose` action.

Check:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo --once /amcl_pose
ros2 action list | grep navigate_to_pose
```

### 5. Launch The Mainloop

Terminal 5:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_mainloop mainloop.launch.py \
  use_sim_time:=false \
  clear_history:=true \
  cmd_vel_topic:=/cmd_vel_nav \
  strafe_block_unknown_costmap:=true \
  strafe_block_timeout_sec:=8.0 \
  blocked_tray_retry_delay_sec:=60.0 \
  plan_path:=~/mdp_ws/generated_row_plan.json
```

To also launch the perception node alongside the mainloop, add:

```bash
ros2 launch mdp_mainloop mainloop.launch.py \
  use_sim_time:=false \
  clear_history:=true \
  cmd_vel_topic:=/cmd_vel_nav \
  strafe_block_unknown_costmap:=true \
  strafe_block_timeout_sec:=8.0 \
  blocked_tray_retry_delay_sec:=60.0 \
  plan_path:=~/mdp_ws/generated_row_plan.json \
  enable_perception:=true \
  perception_device:=cpu
```

Parameter meanings:

```text
enable_perception
  false (default) does not start the perception node.
  true starts mdp_perception alongside the mainloop.
  The mainloop will call /perception/start_scan and /perception/stop_scan
  automatically at the start and end of each SCAN_ROW strafe.

perception_device
  YOLO inference device. Use cpu or cuda:0.
```

Check that the executor has loaded the plan and is waiting:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo --once /amcl_pose
ros2 topic echo --once /mainloop/status
```

### 6. Enable Navigation

Enable autonomy after the robot pose is correct in RViz and the path is clear:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub -r 2 /autonomous_enabled std_msgs/Bool "{data: true}"
```

Leave it running until `/mainloop/status` reports
`"autonomous_enabled": true`, then stop it with `Ctrl+C`.

To pause autonomy:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic pub --once /autonomous_enabled std_msgs/Bool "{data: false}"
```

Useful monitoring commands:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /mission_dashboard
```

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /mainloop/status
```

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo /mainloop/task_result
```

## Simulation Changes For Mission Testing

The real-robot sequence above is the main mission procedure. In simulation,
do not launch `mdp_bringup twist_mux.launch.py`; the Gazebo greenhouse launch
already starts the simulation-side controllers and Gazebo twist mux.

Also, simulation must use the greenhouse setup. Use the greenhouse/asymmetric
map from the simulation package setup, not the final real-robot map.

Change only these commands and parameters for simulation:

```text
Start simulation/navigation/RViz with:
  ros2 launch mdp_navigation sim_nav_loc_rviz.launch.py

use_sim_time:
  false -> true

map:
  /absolute/path/to/real_robot_map.yaml -> mdp_localization/maps/asym_map.yaml
  Do not use the final real-robot map in simulation.

automatic labeller:
  use_sim_time:=true
  map:=$(ros2 pkg prefix mdp_localization)/share/mdp_localization/maps/asym_map.yaml

Nav2 cmd_vel_topic:
  /cmd_vel_nav -> /mirte_base_controller/cmd_vel_unstamped

mainloop cmd_vel_topic:
  /cmd_vel_nav -> /mirte_base_controller/cmd_vel_unstamped

teleop remap:
  /cmd_vel_key or /cmd_vel_joy -> /mirte_base_controller/cmd_vel_unstamped
```

Simulation start command:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_navigation sim_nav_loc_rviz.launch.py \
  use_sim_time:=true \
  map:=$(ros2 pkg prefix mdp_localization)/share/mdp_localization/maps/asym_map.yaml \
  cmd_vel_topic:=/mirte_base_controller/cmd_vel_unstamped
```

Simulation mainloop command:

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
  plan_path:=~/mdp_ws/generated_row_plan.json \
  enable_perception:=true
```

## Mainloop Behavior

The mission executor reads `~/mdp_ws/generated_row_plan.json`.

For each generated tray:

```text
1. Navigate to A with Nav2.
2. Strafe from A to B with direct velocity commands.
3. Navigate to C with Nav2.
4. Strafe from C to D with direct velocity commands.
```

During strafing, the local costmap filters the strafe command. Occupied
cells, and unknown cells when `strafe_block_unknown_costmap:=true`, make
the executor publish zero velocity and wait. If the strafe stays blocked
longer than `strafe_block_timeout_sec`, the tray is skipped until its
retry delay expires.

Mission history is stored in:

```text
~/mdp_ws/mission_history.json
```

Completed segments are skipped only inside the current mission pass.
Restarting with `clear_history:=true` starts fresh.

## Mapping

For simulation mapping, start Gazebo first:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_gazebo greenhouse_world.launch.xml rviz:=false
```

Then run the MDP SLAM launch in another terminal:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch mdp_slam slam_mapping.launch.py
```

For real-robot mapping, start `minimal_master.launch.py` first, then run
SLAM toolbox with wall-clock time:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=$(ros2 pkg prefix mdp_slam)/share/mdp_slam/config/mapping_params_online_async.yaml \
  use_sim_time:=false
```

Save a map:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run nav2_map_server map_saver_cli -f ~/mdp_ws/my_map
```

This creates:

```text
~/mdp_ws/my_map.yaml
~/mdp_ws/my_map.pgm
```

Use the YAML path as the `map:=...` argument for navigation and automatic
tray waypoint labelling.

## Troubleshooting

Check active nodes:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 node list
```

Check the map:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 lifecycle get /map_server
ros2 topic echo --once /map
```

Check localization:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo --once /amcl_pose
ros2 run tf2_ros tf2_echo map odom
ros2 run tf2_ros tf2_echo odom base_link
```

Check Nav2:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 action list | grep navigate_to_pose
ros2 lifecycle get /controller_server
ros2 lifecycle get /planner_server
ros2 lifecycle get /bt_navigator
```

Check the mission executor:

```bash
cd ~/mdp_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic echo --once /mainloop/status
ros2 topic echo /mission_dashboard
```

Common problems:

```text
Map is not visible in RViz
  Check that /map publishes and that the correct map YAML was passed.

Robot does not localize
  Set 2D Pose Estimate in RViz and verify /amcl_pose.

Nav2 plans but does not move
  Check odom -> base_link and the velocity topic.

Real robot does not move
  Check minimal_master.launch.py, mdp_bringup twist_mux, and controllers.
  Nav2/mainloop should publish to /cmd_vel_nav when twist_mux is running.

Automatic tray click is ignored
  Click closer to the black occupied tray region or increase
  click_search_radius_m.

Generated waypoints are too close to the tray
  Increase lateral_offset_m.

Generated scan starts/stops too close to tray ends
  Increase longitudinal_margin_m.

Executor waits in TASK_READY
  Check /autonomous_enabled, /amcl_pose, Nav2 lifecycle state, and the plan file.
```

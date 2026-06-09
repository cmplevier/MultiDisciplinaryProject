"""Mission executor for JSON row plans and planner-provided tasks."""

import json
import math
import os
import random
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


NAV2_SUCCEEDED = 4


class MainLoopNode(Node):
    """Execute one navigation/scan task at a time."""

    def __init__(self):
        super().__init__('mdp_mainloop_node')
        self.get_logger().info('MDP merged mission executor started')

        self.declare_parameter(
            'plan_path',
            '~/mdp_ws/generated_row_plan.json',
        )
        self.declare_parameter('load_plan_file', True)
        self.declare_parameter('require_plan_file', False)
        self.declare_parameter('loop_mission', False)
        self.declare_parameter('history_path', '~/mdp_ws/mission_history.json')
        self.declare_parameter('clear_history', False)

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('cmd_vel_stop_topic', 'cmd_vel_stop')
        self.declare_parameter('cmd_vel_idle_topic', 'cmd_vel_idle')
        self.declare_parameter('task_topic', '/planner/next_task')
        self.declare_parameter('status_topic', '/mainloop/status')
        self.declare_parameter('result_topic', '/mainloop/task_result')
        self.declare_parameter('dashboard_topic', '/mission_dashboard')
        self.declare_parameter('marker_topic', '/mission_markers')
        self.declare_parameter('nav_action_name', 'navigate_to_pose')

        self.declare_parameter('strafe_speed', 0.15)
        self.declare_parameter('strafe_tolerance', 0.15)
        self.declare_parameter('yaw_gain', 0.5)
        self.declare_parameter('max_yaw_rate', 0.5)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('strafe_costmap_enabled', True)
        self.declare_parameter('strafe_costmap_topic', '/local_costmap/costmap')
        self.declare_parameter('strafe_require_costmap', False)
        self.declare_parameter('strafe_costmap_timeout_sec', 1.0)
        self.declare_parameter('strafe_costmap_obstacle_threshold', 65)
        self.declare_parameter('strafe_block_unknown_costmap', True)
        self.declare_parameter('strafe_collision_radius', 0.18)
        self.declare_parameter('strafe_lookahead_time', 1.0)
        self.declare_parameter('strafe_lookahead_step', 0.1)
        self.declare_parameter('strafe_avoidance_x_speed', 0.05)
        self.declare_parameter('strafe_block_timeout_sec', 8.0)
        self.declare_parameter('blocked_tray_selection_mode', 'random_tray')
        self.declare_parameter('blocked_tray_retry_delay_sec', 60.0)

        self.plan_path = self.expand_path('plan_path')
        self.load_plan_file_enabled = self.get_parameter(
            'load_plan_file'
        ).value
        self.require_plan_file = self.get_parameter(
            'require_plan_file'
        ).value
        self.loop_mission = self.get_parameter('loop_mission').value
        self.history_path = self.expand_path('history_path')
        self.clear_history = self.get_parameter('clear_history').value

        self.strafe_speed = self.get_parameter('strafe_speed').value
        self.strafe_tolerance = self.get_parameter(
            'strafe_tolerance'
        ).value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value
        self.pose_timeout_sec = self.get_parameter(
            'pose_timeout_sec'
        ).value
        self.max_retries = self.get_parameter('max_retries').value
        self.strafe_costmap_enabled = self.get_parameter(
            'strafe_costmap_enabled'
        ).value
        self.strafe_require_costmap = self.get_parameter(
            'strafe_require_costmap'
        ).value
        self.strafe_costmap_timeout_sec = self.get_parameter(
            'strafe_costmap_timeout_sec'
        ).value
        self.strafe_costmap_obstacle_threshold = self.get_parameter(
            'strafe_costmap_obstacle_threshold'
        ).value
        self.strafe_block_unknown_costmap = self.get_parameter(
            'strafe_block_unknown_costmap'
        ).value
        self.strafe_collision_radius = self.get_parameter(
            'strafe_collision_radius'
        ).value
        self.strafe_lookahead_time = self.get_parameter(
            'strafe_lookahead_time'
        ).value
        self.strafe_lookahead_step = self.get_parameter(
            'strafe_lookahead_step'
        ).value
        self.strafe_avoidance_x_speed = self.get_parameter(
            'strafe_avoidance_x_speed'
        ).value
        self.strafe_block_timeout_sec = self.get_parameter(
            'strafe_block_timeout_sec'
        ).value
        self.blocked_tray_selection_mode = str(
            self.get_parameter('blocked_tray_selection_mode').value
        ).lower()
        self.blocked_tray_retry_delay_sec = self.get_parameter(
            'blocked_tray_retry_delay_sec'
        ).value

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        stop_topic = self.get_parameter('cmd_vel_stop_topic').value
        idle_topic = self.get_parameter('cmd_vel_idle_topic').value
        task_topic = self.get_parameter('task_topic').value
        status_topic = self.get_parameter('status_topic').value
        result_topic = self.get_parameter('result_topic').value
        dashboard_topic = self.get_parameter('dashboard_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        nav_action_name = self.get_parameter('nav_action_name').value
        strafe_costmap_topic = self.get_parameter(
            'strafe_costmap_topic'
        ).value

        self.history = {}
        self.mission_tasks = []
        self.current_task_index = 0
        self.current_task = None
        self.current_task_is_external = False
        self.current_pose = None
        self.last_pose_time = None
        self.task_start_time = None
        self.autonomous_enabled = False
        self.state = 'WAITING_FOR_TASK'
        self.nav_goal_handle = None
        self.nav_phase = None
        self.canceling_for_pause = False
        self.strafe_start_yaw = 0.0
        self.retry_count = 0
        self.last_status_time = 0.0
        self.plan_loaded = False
        self.completed_task_ids = set()
        self.local_costmap = None
        self.last_costmap_time = None
        self.strafe_blocked_since = None
        self.blocked_trays = {}
        self.next_plan_selection_mode = None

        if self.clear_history and os.path.exists(self.history_path):
            os.remove(self.history_path)
            self.get_logger().info('Mission history cleared')

        self.load_history()
        if self.load_plan_file_enabled:
            self.mission_tasks = self.load_plan_file(self.plan_path)
            self.plan_loaded = bool(self.mission_tasks)
            self.current_task_index = self.get_first_incomplete_task()

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            nav_action_name,
        )

        self.perception_start_client = self.create_client(
            Trigger, '/perception/start_scan'
        )
        self.perception_stop_client = self.create_client(
            Trigger, '/perception/stop_scan'
        )

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.stop_pub = self.create_publisher(Twist, stop_topic, 10)
        self.idle_pub = self.create_publisher(Twist, idle_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.dashboard_pub = self.create_publisher(
            String,
            dashboard_topic,
            10,
        )
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        amcl_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            amcl_qos,
        )
        self.enable_sub = self.create_subscription(
            Bool,
            '/autonomous_enabled',
            self.enable_callback,
            10,
        )
        self.task_sub = self.create_subscription(
            String,
            task_topic,
            self.task_callback,
            10,
        )
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            strafe_costmap_topic,
            self.costmap_callback,
            10,
        )

        self.timer = self.create_timer(0.05, self.control_loop)
        self.publish_status(force=True)

    def expand_path(self, parameter_name):
        """Expand a string path parameter."""
        value = self.get_parameter(parameter_name).value
        return os.path.expanduser(str(value))

    def load_history(self):
        """Load the persistent run log from disk."""
        if not os.path.exists(self.history_path):
            self.history = {}
            return

        try:
            with open(self.history_path, 'r', encoding='utf-8') as handle:
                self.history = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'Could not load mission history: {exc}; starting fresh'
            )
            self.history = {}

    def save_history(self, task, duration):
        """Persist a successful task without making it permanently ineligible."""
        task_id = task['task_id']
        previous = self.history.get(task_id, {})
        try:
            previous_count = previous.get('completed_count')
            if previous_count is None and 'completed_at' in previous:
                previous_count = 1
            completed_count = int(previous_count or 0) + 1
        except (TypeError, ValueError):
            completed_count = 1

        self.history[task_id] = {
            'completed_count': completed_count,
            'last_completed_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'duration': f'{duration:.1f}s',
            'last_duration': f'{duration:.1f}s',
            'task_type': task['type'],
            'row_id': task.get('row_id'),
            'tray_id': task.get('tray_id'),
            'segment_id': task.get('segment_id'),
        }
        self.completed_task_ids.add(task_id)

        try:
            with open(self.history_path, 'w', encoding='utf-8') as handle:
                json.dump(self.history, handle, indent=4)
        except OSError as exc:
            self.get_logger().error(f'Failed to save mission history: {exc}')

    def load_plan_file(self, plan_path):
        """Read the row plan JSON and normalize it into executable tasks."""
        if not os.path.exists(plan_path):
            message = f'Plan file does not exist: {plan_path}'
            if self.require_plan_file:
                self.get_logger().error(message)
            else:
                self.get_logger().warn(
                    f'{message}; waiting for tasks on /planner/next_task'
                )
            return []

        try:
            with open(plan_path, 'r', encoding='utf-8') as handle:
                raw_plan = json.load(handle)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'Could not read plan file: {exc}')
            return []

        tasks = self.parse_plan(raw_plan)
        if tasks:
            self.get_logger().info(
                f'Loaded {len(tasks)} task(s) from {plan_path}'
            )
        else:
            self.get_logger().warn('Plan file has no executable tasks')
        return tasks

    def parse_plan(self, raw_plan):
        """Accept tray-plan, row-plan, task-list, and old segment formats."""
        if isinstance(raw_plan, list):
            raw_tasks = raw_plan
            raw_trays = []
            return_home_pose = None
        elif isinstance(raw_plan, dict):
            raw_trays = raw_plan.get('trays') or []
            raw_tasks = (
                raw_plan.get('tasks')
                or raw_plan.get('rows')
                or raw_plan.get('mission_segments')
                or []
            )
            return_home_pose = raw_plan.get('return_home_pose')
        else:
            self.get_logger().error('Plan JSON must be an object or list')
            return []

        tasks = []
        for index, raw_tray in enumerate(raw_trays):
            tasks.extend(self.parse_tray_object(raw_tray, index + 1))

        for index, raw_task in enumerate(raw_tasks):
            task = self.parse_task_object(raw_task, f'task_{index + 1}')
            if task is not None:
                tasks.append(task)

        if return_home_pose is not None:
            home_task = self.parse_task_object(
                {
                    'task_id': 'return_home',
                    'type': 'NAV_ONLY',
                    'goal_pose': return_home_pose,
                },
                'return_home',
            )
            if home_task is not None:
                tasks.append(home_task)

        return tasks

    def parse_tray_object(self, raw_tray, tray_number):
        """Expand one tray definition into A->B and C->D scan tasks."""
        if not isinstance(raw_tray, dict):
            self.get_logger().error('Each tray must be a JSON object')
            return []

        tray_id = str(
            raw_tray.get('tray_id') or raw_tray.get('id') or
            f'tray_{tray_number}'
        )
        frame_id = raw_tray.get('frame_id', 'map')

        rows = raw_tray.get('rows')
        if rows is not None:
            return self.parse_tray_rows(rows, tray_id, frame_id)

        waypoints = self.parse_tray_waypoints(raw_tray)
        if waypoints is None:
            self.get_logger().error(
                f'Tray {tray_id} needs waypoints A/B/C/D or rows'
            )
            return []

        tasks = []
        for start_key, end_key in [('A', 'B'), ('C', 'D')]:
            if start_key not in waypoints or end_key not in waypoints:
                self.get_logger().error(
                    f'Tray {tray_id} is missing {start_key}/{end_key}'
                )
                continue
            segment_id = f'{start_key}_to_{end_key}'
            task = self.parse_task_object(
                {
                    'task_id': f'{tray_id}_{segment_id}',
                    'type': 'SCAN_ROW',
                    'row_id': f'{tray_id}_{segment_id}',
                    'tray_id': tray_id,
                    'segment_id': segment_id,
                    'frame_id': frame_id,
                    'approach_pose': waypoints[start_key],
                    'scan_end_pose': waypoints[end_key],
                },
                f'{tray_id}_{segment_id}',
            )
            if task is not None:
                tasks.append(task)

        return tasks

    def parse_tray_rows(self, rows, tray_id, frame_id):
        """Parse tray rows already expressed as scan-row objects."""
        if not isinstance(rows, list):
            self.get_logger().error(f'Tray {tray_id} rows must be a list')
            return []

        tasks = []
        for index, raw_row in enumerate(rows):
            if not isinstance(raw_row, dict):
                self.get_logger().error(
                    f'Tray {tray_id} row {index + 1} must be an object'
                )
                continue
            row = dict(raw_row)
            segment_id = str(
                row.get('segment_id') or row.get('id') or
                f'row_{index + 1}'
            )
            row.setdefault('type', 'SCAN_ROW')
            row.setdefault('task_id', f'{tray_id}_{segment_id}')
            row.setdefault('row_id', f'{tray_id}_{segment_id}')
            row.setdefault('tray_id', tray_id)
            row.setdefault('segment_id', segment_id)
            row.setdefault('frame_id', frame_id)
            task = self.parse_task_object(row, f'{tray_id}_{segment_id}')
            if task is not None:
                tasks.append(task)

        return tasks

    def parse_tray_waypoints(self, raw_tray):
        """Return a waypoint-name to pose mapping for a tray."""
        raw_waypoints = (
            raw_tray.get('waypoints')
            or raw_tray.get('poses')
            or raw_tray.get('points')
        )
        if raw_waypoints is None:
            return None

        waypoints = {}
        if isinstance(raw_waypoints, dict):
            for name, raw_pose in raw_waypoints.items():
                pose = self.parse_pose(raw_pose)
                if pose is None:
                    return None
                waypoints[str(name).upper()] = pose
        elif isinstance(raw_waypoints, list):
            if len(raw_waypoints) < 4:
                return None
            for name, raw_pose in zip(['A', 'B', 'C', 'D'], raw_waypoints):
                pose = self.parse_pose(raw_pose)
                if pose is None:
                    return None
                waypoints[name] = pose
        else:
            return None

        return waypoints

    def get_first_incomplete_task(self):
        """Return the first plan index that is not in mission history."""
        for index, task in enumerate(self.mission_tasks):
            if task['task_id'] not in self.completed_task_ids:
                return index
        return len(self.mission_tasks)

    def task_callback(self, msg):
        """Accept a JSON task from a planner when this node is available."""
        task = self.parse_task_json(msg.data)
        if task is None:
            return

        if self.current_task is not None:
            current_id = self.current_task['task_id']
            if task['task_id'] != current_id:
                self.get_logger().warn(
                    f"Ignoring task {task['task_id']}; "
                    f'executor is busy with {current_id}'
                )
            return

        if self.has_pending_plan_task():
            self.get_logger().warn(
                'Ignoring planner task because a plan file mission is active. '
                'Launch with load_plan_file:=false to use /planner/next_task.'
            )
            return

        self.current_task = task
        self.current_task_is_external = True
        self.task_start_time = None
        self.retry_count = 0
        self.nav_phase = None
        self.state = 'TASK_READY'
        self.get_logger().info(
            f"Accepted external task {task['task_id']} ({task['type']})"
        )
        self.publish_status(force=True)

    def parse_task_json(self, raw_data):
        """Parse a JSON task message."""
        try:
            raw_task = json.loads(raw_data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid task JSON: {exc}')
            return None

        return self.parse_task_object(raw_task, 'external_task')

    def parse_task_object(self, raw_task, default_task_id):
        """Normalize one raw task/row dictionary."""
        if not isinstance(raw_task, dict):
            self.get_logger().error('Each task must be a JSON object')
            return None

        raw_type = str(raw_task.get('type', 'SCAN_ROW')).upper()
        task_type = self.normalize_task_type(raw_type)
        task_id = (
            raw_task.get('task_id')
            or raw_task.get('id')
            or raw_task.get('row_id')
            or default_task_id
        )
        row_id = raw_task.get('row_id') or raw_task.get('id')
        tray_id = raw_task.get('tray_id') or raw_task.get('tray')
        segment_id = (
            raw_task.get('segment_id')
            or raw_task.get('segment')
            or raw_task.get('edge_id')
        )

        task = {
            'task_id': str(task_id),
            'type': task_type,
            'row_id': row_id,
            'frame_id': raw_task.get('frame_id', 'map'),
        }
        if tray_id is not None:
            task['tray_id'] = str(tray_id)
        if segment_id is not None:
            task['segment_id'] = str(segment_id)

        if task_type == 'SCAN_ROW':
            approach_pose = self.parse_pose(raw_task.get('approach_pose'))
            scan_end_pose = self.parse_pose(
                raw_task.get('scan_end_pose')
                or raw_task.get('goal_pose')
                or raw_task.get('end_pose')
                or raw_task.get('target_pose')
            )
            if approach_pose is None or scan_end_pose is None:
                self.get_logger().error(
                    f"SCAN_ROW task {task_id} needs approach_pose "
                    'and scan_end_pose'
                )
                return None
            task['approach_pose'] = approach_pose
            task['scan_end_pose'] = scan_end_pose
        elif task_type == 'NAV_ONLY':
            goal_pose = self.parse_pose(
                raw_task.get('goal_pose')
                or raw_task.get('target_pose')
                or raw_task.get('approach_pose')
                or raw_task.get('target')
            )
            if goal_pose is None:
                self.get_logger().error(
                    f'NAV_ONLY task {task_id} needs goal_pose'
                )
                return None
            task['goal_pose'] = goal_pose
        elif task_type == 'STRAFE_ONLY':
            scan_end_pose = self.parse_pose(
                raw_task.get('scan_end_pose')
                or raw_task.get('goal_pose')
                or raw_task.get('end_pose')
                or raw_task.get('target_pose')
                or raw_task.get('target')
            )
            if scan_end_pose is None:
                self.get_logger().error(
                    f'STRAFE_ONLY task {task_id} needs scan_end_pose'
                )
                return None
            task['scan_end_pose'] = scan_end_pose
        else:
            self.get_logger().error(f'Unsupported task type: {raw_type}')
            return None

        return task

    @staticmethod
    def normalize_task_type(raw_type):
        """Map old names and row-plan names onto executor task types."""
        if raw_type == 'NAV':
            return 'NAV_ONLY'
        if raw_type == 'STRAFE':
            return 'STRAFE_ONLY'
        return raw_type

    @staticmethod
    def parse_pose(raw_pose):
        """Return [x, y, yaw] from list-style or dict-style pose data."""
        if isinstance(raw_pose, (list, tuple)):
            if len(raw_pose) < 2:
                return None
            try:
                x = float(raw_pose[0])
                y = float(raw_pose[1])
                yaw = None if len(raw_pose) < 3 else float(raw_pose[2])
            except (TypeError, ValueError):
                return None
            return [x, y, yaw]

        if isinstance(raw_pose, dict):
            try:
                position = raw_pose.get('position', {})
                x = raw_pose.get('x', position.get('x'))
                y = raw_pose.get('y', position.get('y'))
                yaw = raw_pose.get('yaw', raw_pose.get('theta'))
                if yaw is None:
                    yaw = MainLoopNode.yaw_from_pose_dict(raw_pose)
                yaw = None if yaw is None else float(yaw)
                return [float(x), float(y), yaw]
            except (AttributeError, TypeError, ValueError):
                return None

        return None

    @staticmethod
    def yaw_from_pose_dict(raw_pose):
        """Extract yaw from a dict containing quaternion orientation."""
        orientation = raw_pose.get('orientation')
        if not isinstance(orientation, dict):
            return None

        try:
            x = float(orientation.get('x', 0.0))
            y = float(orientation.get('y', 0.0))
            z = float(orientation.get('z', 0.0))
            w = float(orientation.get('w', 1.0))
        except (TypeError, ValueError):
            return None

        return math.atan2(
            2 * (w * z + x * y),
            1 - 2 * (y * y + z * z),
        )

    def pose_callback(self, msg):
        """Store the latest AMCL pose."""
        self.current_pose = msg.pose.pose
        self.last_pose_time = self.get_clock().now()

    def costmap_callback(self, msg):
        """Store the latest local costmap for strafe safety checks."""
        self.local_costmap = msg
        self.last_costmap_time = self.get_clock().now()

    def enable_callback(self, msg):
        """Pause or resume autonomous execution."""
        was_enabled = self.autonomous_enabled
        self.autonomous_enabled = msg.data

        if self.autonomous_enabled and not was_enabled:
            self.get_logger().info('>>> Autonomous Mode: ENABLED')
            if self.state == 'ERROR':
                self.get_logger().warn(
                    'Executor is in ERROR; restart or clear the issue first'
                )
            elif self.current_task is not None:
                self.state = 'TASK_READY'
            elif self.has_pending_plan_task():
                self.state = 'START_NEXT_TASK'
            else:
                self.state = 'WAITING_FOR_TASK'
        elif not self.autonomous_enabled and was_enabled:
            self.get_logger().info('<<< Autonomous Mode: DISABLED')
            self.force_stop_robot()
            if self.nav_goal_handle is not None:
                self.canceling_for_pause = True
                self.get_logger().info('Canceling active Nav2 goal...')
                self.nav_goal_handle.cancel_goal_async()
            if self.current_task is not None:
                self.state = 'PAUSED'
            elif self.state != 'MISSION_COMPLETE':
                self.state = 'IDLE'

        self.publish_status(force=True)

    def control_loop(self):
        """Advance the executor state machine."""
        self.publish_markers()
        self.publish_dashboard()
        self.publish_status()

        self.idle_pub.publish(Twist())

        if not self.autonomous_enabled:
            self.stop_pub.publish(Twist())
            return

        if self.state in ('ERROR', 'MISSION_COMPLETE'):
            self.force_stop_robot()
            return

        if not self.pose_is_ready():
            return

        if self.current_task is None:
            if self.has_pending_plan_task():
                self.start_next_plan_task()
            else:
                self.state = 'WAITING_FOR_TASK'
            return

        if self.state == 'TASK_READY':
            self.start_current_task()
        elif self.state == 'STRAFING_ROW':
            self.perform_strafe()

    def pose_is_ready(self):
    #     """Check whether AMCL pose exists and is fresh enough."""
    #     if self.current_pose is None or self.last_pose_time is None:
    #         self.get_logger().warn(
    #             'Waiting for robot pose on /amcl_pose...',
    #             throttle_duration_sec=2.0,
    #         )
    #         return False

    #     if self.pose_timeout_sec <= 0.0:
    #         return True

    #     age = (
    #         self.get_clock().now() - self.last_pose_time
    #     ).nanoseconds / 1e9
    #     if age > self.pose_timeout_sec:
    #         self.get_logger().warn(
    #             f'Robot pose is stale ({age:.2f}s old)',
    #             throttle_duration_sec=2.0,
    #         )
    #         return False

        return True

    def has_pending_plan_task(self):
        """Return true when the saved plan still has incomplete tasks."""
        return any(
            task['task_id'] not in self.completed_task_ids
            for task in self.mission_tasks
        )

    def start_next_plan_task(self):
        """Load the next incomplete task from the saved plan."""
        self.release_expired_blocked_trays()
        task_index = self.choose_next_plan_task_index()
        if (
            task_index is None
            and self.blocked_trays
            and self.has_revisitable_unblocked_task()
        ):
            self.get_logger().info(
                'No unfinished unblocked tray tasks remain; starting '
                'another pass while blocked trays cool down.'
            )
            self.completed_task_ids.clear()
            self.current_task_index = 0
            task_index = self.choose_next_plan_task_index()

        if task_index is None and self.blocked_trays:
            self.get_logger().info(
                'No unblocked tray tasks remain; retrying blocked trays.'
            )
            self.blocked_trays.clear()
            task_index = self.choose_next_plan_task_index()

        if task_index is None:
            self.handle_mission_complete()
            return

        self.current_task_index = task_index
        task = self.mission_tasks[task_index]
        self.current_task = task
        self.current_task_is_external = False
        self.task_start_time = None
        self.retry_count = 0
        self.nav_phase = None
        self.strafe_blocked_since = None
        self.state = 'TASK_READY'
        self.get_logger().info(
            f"Selected plan task {task['task_id']} ({task['type']})"
        )
        self.publish_status(force=True)

    def choose_next_plan_task_index(self):
        """Choose the next incomplete task, respecting temporary tray blocks."""
        selection_mode = self.next_plan_selection_mode
        eligible = self.eligible_plan_task_indices()
        if not eligible:
            return None

        self.next_plan_selection_mode = None
        if selection_mode in ('random', 'random_tray'):
            return self.choose_random_tray_task_index(eligible)

        ordered = sorted(eligible)
        for index in ordered:
            if index >= self.current_task_index:
                return index
        return ordered[0]

    def choose_random_tray_task_index(self, eligible):
        """Choose a random tray, then its first eligible segment."""
        by_tray = {}
        no_tray = []
        for index in eligible:
            tray_id = self.mission_tasks[index].get('tray_id')
            if tray_id is None:
                no_tray.append(index)
            else:
                by_tray.setdefault(tray_id, []).append(index)

        if by_tray:
            tray_id = random.choice(list(by_tray.keys()))
            return min(by_tray[tray_id])

        return random.choice(no_tray or eligible)

    def eligible_plan_task_indices(self):
        """Return incomplete task indices allowed to run now."""
        return [
            index
            for index, task in enumerate(self.mission_tasks)
            if self.plan_task_is_eligible(task)
        ]

    def plan_task_is_eligible(self, task):
        """Return true when a saved-plan task can run now."""
        if task['task_id'] in self.completed_task_ids:
            return False

        tray_id = task.get('tray_id')
        if tray_id is not None and tray_id in self.blocked_trays:
            return False

        if self.is_return_home_task(task) and self.has_incomplete_scan_tasks():
            return False

        return True

    def has_incomplete_scan_tasks(self):
        """Return true while scan/strafe tasks remain incomplete."""
        for task in self.mission_tasks:
            if task['task_id'] in self.completed_task_ids:
                continue
            if task['type'] in ('SCAN_ROW', 'STRAFE_ONLY'):
                return True
        return False

    def has_revisitable_unblocked_task(self):
        """Return true if a completed non-blocked scan can run again."""
        for task in self.mission_tasks:
            if task['task_id'] not in self.completed_task_ids:
                continue
            if task['type'] not in ('SCAN_ROW', 'STRAFE_ONLY'):
                continue
            tray_id = task.get('tray_id')
            if tray_id is not None and tray_id in self.blocked_trays:
                continue
            return True
        return False

    def release_expired_blocked_trays(self):
        """Make temporarily blocked trays eligible after their cooldown."""
        if not self.blocked_trays:
            return

        if self.blocked_tray_retry_delay_sec <= 0.0:
            released = sorted(self.blocked_trays.keys())
            self.blocked_trays.clear()
        else:
            now = time.time()
            released = []
            for tray_id, block in list(self.blocked_trays.items()):
                retry_after = block.get('retry_after_time')
                if retry_after is None:
                    blocked_at = block.get('blocked_at_time', now)
                    retry_after = blocked_at + self.blocked_tray_retry_delay_sec
                if now >= retry_after:
                    released.append(tray_id)
                    self.blocked_trays.pop(tray_id, None)

        if released:
            self.get_logger().info(
                'Re-enabled blocked tray/trays after cooldown: '
                + ', '.join(released)
            )

    @staticmethod
    def is_return_home_task(task):
        """Return true for the optional final return-home task."""
        return task['type'] == 'NAV_ONLY' and task['task_id'] == 'return_home'

    def handle_mission_complete(self):
        """Stop or restart after all saved-plan tasks are complete."""
        if self.loop_mission and self.mission_tasks:
            self.get_logger().info(
                'Mission cycle complete; restarting without clearing history'
            )
            self.completed_task_ids.clear()
            self.blocked_trays.clear()
            self.current_task_index = 0
            self.state = 'START_NEXT_TASK'
            return

        if self.state != 'MISSION_COMPLETE':
            self.get_logger().info('Mission complete')
        self.state = 'MISSION_COMPLETE'
        self.force_stop_robot()
        self.publish_status(force=True)

    def start_current_task(self):
        """Start or restart the active task."""
        if self.current_task is None:
            return

        if self.task_start_time is None:
            self.task_start_time = time.time()

        task = self.current_task
        if task['type'] == 'SCAN_ROW':
            self.get_logger().info(
                f"Task Start: {task['task_id']} "
                '(navigating to scan start)'
            )
            self.state = 'NAVIGATING_TO_APPROACH'
            self.start_nav_to_pose(task['approach_pose'], 'approach')
        elif task['type'] == 'NAV_ONLY':
            self.get_logger().info(
                f"Task Start: {task['task_id']} (navigating)"
            )
            self.state = 'NAVIGATING_TO_GOAL'
            self.start_nav_to_pose(task['goal_pose'], 'goal')
        elif task['type'] == 'STRAFE_ONLY':
            self.get_logger().info(
                f"Task Start: {task['task_id']} (strafing)"
            )
            self.strafe_start_yaw = self.get_yaw_from_pose(self.current_pose)
            self.strafe_blocked_since = None
            self.state = 'STRAFING_ROW'

        self.publish_status(force=True)

    def start_nav_to_pose(self, pose, phase):
        """Send a Nav2 NavigateToPose goal."""
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.current_task.get('frame_id', 'map')
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(pose[0])
        goal.pose.pose.position.y = float(pose[1])

        yaw = pose[2]
        if yaw is not None:
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation = self.current_pose.orientation

        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 action server unavailable')
            self.handle_task_failure('nav2_action_server_unavailable')
            return

        self.nav_phase = phase
        send_goal_future = self.nav_client.send_goal_async(goal)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle Nav2 accepting or rejecting the goal."""
        if self.current_task is None:
            return

        try:
            self.nav_goal_handle = future.result()
        except Exception as exc:  # noqa: B902
            self.get_logger().error(f'Nav2 goal request failed: {exc}')
            self.handle_task_failure('nav_goal_request_failed')
            return

        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self.handle_task_failure('nav_goal_rejected')
            return

        if not self.autonomous_enabled:
            self.canceling_for_pause = True
            self.nav_goal_handle.cancel_goal_async()
            return

        result_future = self.nav_goal_handle.get_result_async()
        result_future.add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        """Handle the final result from Nav2."""
        if self.current_task is None:
            return

        try:
            result = future.result()
            status = result.status
        except Exception as exc:  # noqa: B902
            self.get_logger().error(f'Nav2 result failed: {exc}')
            self.handle_task_failure('nav_result_failed')
            return

        self.nav_goal_handle = None
        phase = self.nav_phase
        self.nav_phase = None

        if self.canceling_for_pause or not self.autonomous_enabled:
            self.canceling_for_pause = False
            self.state = 'PAUSED'
            self.force_stop_robot()
            self.get_logger().info('Navigation paused')
            self.publish_status(force=True)
            return

        if status != NAV2_SUCCEEDED:
            self.get_logger().warn(f'Nav2 ended with status {status}')
            self.handle_task_failure(f'nav_status_{status}')
            return

        if phase == 'approach' and self.current_task['type'] == 'SCAN_ROW':
            self.get_logger().info(
                f"Reached scan start for {self.current_task['task_id']}"
            )
            self._call_perception(self.perception_start_client, '/perception/start_scan')
            self.strafe_start_yaw = self.get_yaw_from_pose(self.current_pose)
            self.strafe_blocked_since = None
            self.state = 'STRAFING_ROW'
            self.publish_status(force=True)
            return

        self.finish_current_task(True, 'completed')

    def perform_strafe(self):
        """Drive in the robot frame toward the row scan end pose."""
        target = self.current_task['scan_end_pose']
        dx = target[0] - self.current_pose.position.x
        dy = target[1] - self.current_pose.position.y
        distance = math.sqrt(dx ** 2 + dy ** 2)

        if distance < self.strafe_tolerance:
            self.get_logger().info(
                f"Task Complete: {self.current_task['task_id']}"
            )
            self.force_stop_robot()
            self._call_perception(self.perception_stop_client, '/perception/stop_scan')
            self.finish_current_task(True, 'completed')
            return

        yaw = self.get_yaw_from_pose(self.current_pose)
        local_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        local_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        target_yaw = target[2]
        if target_yaw is None:
            target_yaw = self.strafe_start_yaw
        yaw_error = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()
        twist.linear.x = self.clamp(
            local_x * self.strafe_speed,
            -self.strafe_speed,
            self.strafe_speed,
        )
        twist.linear.y = self.clamp(
            local_y * self.strafe_speed,
            -self.strafe_speed,
            self.strafe_speed,
        )
        twist.angular.z = self.clamp(
            yaw_error * self.yaw_gain,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )

        command, safety_note = self.apply_strafe_costmap_safety(twist, yaw)
        if command is None:
            self.handle_blocked_strafe(safety_note)
            return

        self.strafe_blocked_since = None
        self.get_logger().info(
            f'Strafing: dist={distance:.2f}m, '
            f'local_err=({local_x:.2f}, {local_y:.2f}), '
            f'cmd=({command.linear.x:.2f}, {command.linear.y:.2f}), '
            f'safety={safety_note}',
            throttle_duration_sec=1.0,
        )
        self.cmd_vel_pub.publish(command)

    def apply_strafe_costmap_safety(self, desired_twist, yaw):
        """Return a safe strafe command while preserving lateral preference."""
        if not self.strafe_costmap_enabled:
            return desired_twist, 'costmap_disabled'

        costmap_status = self.costmap_status_for_strafe()
        if costmap_status is not None:
            if self.strafe_require_costmap:
                return None, costmap_status
            self.get_logger().warn(
                f'Strafe costmap check skipped: {costmap_status}',
                throttle_duration_sec=2.0,
            )
            return desired_twist, costmap_status

        for candidate, label in self.strafe_twist_candidates(desired_twist):
            if self.strafe_twist_is_safe(candidate, yaw):
                return candidate, label

        return None, 'blocked_by_local_costmap'

    def costmap_status_for_strafe(self):
        """Return a problem description when the local costmap is unusable."""
        if self.local_costmap is None or self.last_costmap_time is None:
            return 'waiting_for_local_costmap'

        if self.strafe_costmap_timeout_sec > 0.0:
            age = (
                self.get_clock().now() - self.last_costmap_time
            ).nanoseconds / 1e9
            if age > self.strafe_costmap_timeout_sec:
                return f'local_costmap_stale_{age:.2f}s'

        costmap_frame = self.local_costmap.header.frame_id.lstrip('/')
        task_frame = self.current_task.get('frame_id', 'map').lstrip('/')
        if costmap_frame and task_frame and costmap_frame != task_frame:
            return f'costmap_frame_{costmap_frame}_not_{task_frame}'

        info = self.local_costmap.info
        if info.width <= 0 or info.height <= 0 or info.resolution <= 0.0:
            return 'local_costmap_has_invalid_metadata'

        return None

    def strafe_twist_candidates(self, desired_twist):
        """Generate commands ordered from most strafe-like to most cautious."""
        max_speed = abs(self.strafe_speed)
        avoid_speed = abs(self.strafe_avoidance_x_speed)
        base_x = desired_twist.linear.x
        base_y = desired_twist.linear.y
        base_w = desired_twist.angular.z
        seen = set()
        candidates = []

        def add(vx, vy, label):
            vx = self.clamp(vx, -max_speed, max_speed)
            vy = self.clamp(vy, -max_speed, max_speed)
            key = (round(vx, 4), round(vy, 4), round(base_w, 4))
            if key in seen:
                return
            seen.add(key)
            candidates.append((self.make_twist(vx, vy, base_w), label))

        add(base_x, base_y, 'desired_strafe')

        if abs(base_x) > 0.01:
            first_sign = math.copysign(1.0, base_x)
        else:
            first_sign = 1.0
        correction_signs = (first_sign, -first_sign)

        for scale in (0.75, 0.5, 0.25):
            add(base_x, base_y * scale, f'slow_strafe_{scale:.2f}')
            for sign in correction_signs:
                add(
                    base_x + sign * avoid_speed,
                    base_y * scale,
                    f'avoid_x_{sign * avoid_speed:.2f}_strafe_{scale:.2f}',
                )

        return candidates

    def strafe_twist_is_safe(self, twist, yaw):
        """Check whether a command keeps the robot footprint off obstacles."""
        speed = math.hypot(twist.linear.x, twist.linear.y)
        if speed < 1e-4:
            return True

        horizon = max(0.0, self.strafe_lookahead_time)
        step = max(0.02, self.strafe_lookahead_step)
        steps = max(1, int(math.ceil(horizon / step)))

        start_x = self.current_pose.position.x
        start_y = self.current_pose.position.y
        map_vx = twist.linear.x * math.cos(yaw) - twist.linear.y * math.sin(yaw)
        map_vy = twist.linear.x * math.sin(yaw) + twist.linear.y * math.cos(yaw)

        for index in range(1, steps + 1):
            dt = min(index * step, horizon)
            if dt <= 0.0:
                dt = step
            x = start_x + map_vx * dt
            y = start_y + map_vy * dt
            if not self.footprint_position_is_safe(x, y):
                return False

        return True

    def footprint_position_is_safe(self, x, y):
        """Check a circular robot footprint against the occupancy grid."""
        cell = self.world_to_costmap_cell(x, y)
        if cell is None:
            return False

        center_x, center_y = cell
        resolution = self.local_costmap.info.resolution
        radius_cells = max(
            1,
            int(math.ceil(self.strafe_collision_radius / resolution)),
        )

        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                distance = math.hypot(offset_x, offset_y) * resolution
                if distance > self.strafe_collision_radius:
                    continue
                if self.costmap_cell_is_occupied(
                    center_x + offset_x,
                    center_y + offset_y,
                ):
                    return False

        return True

    def world_to_costmap_cell(self, x, y):
        """Convert a world pose in the costmap frame to grid indices."""
        info = self.local_costmap.info
        origin = info.origin
        dx = x - origin.position.x
        dy = y - origin.position.y
        origin_yaw = self.get_yaw_from_pose(origin)
        local_x = dx * math.cos(origin_yaw) + dy * math.sin(origin_yaw)
        local_y = -dx * math.sin(origin_yaw) + dy * math.cos(origin_yaw)
        mx = int(math.floor(local_x / info.resolution))
        my = int(math.floor(local_y / info.resolution))

        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return None
        return mx, my

    def costmap_cell_is_occupied(self, mx, my):
        """Return true if a costmap cell should block strafe motion."""
        info = self.local_costmap.info
        if mx < 0 or my < 0 or mx >= info.width or my >= info.height:
            return True

        value = self.local_costmap.data[my * info.width + mx]
        if value < 0:
            return self.strafe_block_unknown_costmap
        return value >= self.strafe_costmap_obstacle_threshold

    def handle_blocked_strafe(self, reason):
        """Stop during strafe when no safe costmap-aware command exists."""
        now = time.time()
        if self.strafe_blocked_since is None:
            self.strafe_blocked_since = now

        zero = Twist()
        self.cmd_vel_pub.publish(zero)
        self.stop_pub.publish(zero)

        blocked_for = now - self.strafe_blocked_since
        self.get_logger().warn(
            f'Strafe blocked ({reason}); holding position '
            f'for {blocked_for:.1f}s',
            throttle_duration_sec=1.0,
        )

        if (
            self.strafe_block_timeout_sec > 0.0
            and blocked_for >= self.strafe_block_timeout_sec
        ):
            self.skip_current_tray_after_blocked_strafe(reason)

    def skip_current_tray_after_blocked_strafe(self, reason):
        """Temporarily block the current tray and keep the mission moving."""
        if self.current_task is None:
            return

        tray_id = self.current_task.get('tray_id')
        if tray_id is None:
            self.handle_task_failure(reason)
            return

        task = self.current_task
        duration = self.current_task_duration()
        was_external = self.current_task_is_external
        self.force_stop_robot()
        self.publish_task_result(
            False,
            f'tray_temporarily_blocked:{reason}',
            duration,
        )

        blocked_at_time = time.time()
        self.blocked_trays[tray_id] = {
            'reason': reason,
            'task_id': task['task_id'],
            'blocked_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'blocked_at_time': blocked_at_time,
            'retry_after_time': (
                blocked_at_time + self.blocked_tray_retry_delay_sec
            ),
        }
        self.get_logger().warn(
            f"Tray {tray_id} temporarily blocked by {task['task_id']} "
            f'({reason}); selecting another tray.'
        )

        self.current_task = None
        self.current_task_is_external = False
        self.task_start_time = None
        self.nav_phase = None
        self.canceling_for_pause = False
        self.strafe_blocked_since = None
        self.retry_count = 0
        self.next_plan_selection_mode = self.blocked_tray_selection_mode

        if was_external:
            self.state = (
                'WAITING_FOR_TASK' if self.autonomous_enabled else 'IDLE'
            )
        elif self.autonomous_enabled and self.has_pending_plan_task():
            self.state = 'START_NEXT_TASK'
        elif self.autonomous_enabled:
            self.handle_mission_complete()
        else:
            self.state = 'IDLE'

        self.publish_status(force=True)

    @staticmethod
    def make_twist(vx, vy, wz):
        """Create a planar Twist command."""
        twist = Twist()
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = wz
        return twist

    def handle_task_failure(self, reason):
        """Retry a failed task until max_retries is reached."""
        self.force_stop_robot()
        self.nav_goal_handle = None
        self.nav_phase = None
        self.strafe_blocked_since = None
        self.retry_count += 1

        if self.retry_count >= self.max_retries:
            task_id = self.current_task['task_id']
            self.get_logger().error(
                f'Max retries reached for {task_id}; disabling autonomy'
            )
            self.publish_task_result(False, reason)
            self.state = 'ERROR'
            self.autonomous_enabled = False
            self.publish_status(force=True)
            return

        self.get_logger().info(
            f'Retrying task ({self.retry_count}/{self.max_retries})...'
        )
        self.state = 'TASK_READY'
        self.publish_status(force=True)

    def finish_current_task(self, success, reason):
        """Publish result, persist success, and advance the mission."""
        if self.current_task is None:
            return

        self.force_stop_robot()
        duration = self.current_task_duration()
        task = self.current_task
        was_external = self.current_task_is_external

        if success:
            self.save_history(task, duration)

        self.publish_task_result(success, reason, duration)

        if success:
            self.get_logger().info(
                f"Task Result: {task['task_id']} succeeded"
            )
        else:
            self.get_logger().warn(
                f"Task Result: {task['task_id']} failed ({reason})"
            )

        if success and not was_external:
            self.current_task_index += 1

        self.current_task = None
        self.current_task_is_external = False
        self.task_start_time = None
        self.nav_phase = None
        self.canceling_for_pause = False
        self.strafe_blocked_since = None
        self.retry_count = 0

        if self.autonomous_enabled and was_external:
            self.state = 'WAITING_FOR_TASK'
        elif self.autonomous_enabled and self.has_pending_plan_task():
            self.state = 'START_NEXT_TASK'
        elif self.autonomous_enabled:
            self.handle_mission_complete()
        else:
            self.state = 'IDLE'

        self.publish_status(force=True)

    def publish_task_result(self, success, reason, duration=None):
        """Publish a JSON task result message."""
        if self.current_task is None:
            return

        if duration is None:
            duration = self.current_task_duration()

        task = self.current_task
        result = {
            'task_id': task['task_id'],
            'row_id': task.get('row_id'),
            'tray_id': task.get('tray_id'),
            'segment_id': task.get('segment_id'),
            'task_type': task['type'],
            'success': bool(success),
            'reason': reason,
            'duration': round(duration, 2),
        }
        msg = String()
        msg.data = json.dumps(result)
        self.result_pub.publish(msg)

    def current_task_duration(self):
        """Return seconds since the current task first started."""
        if self.task_start_time is None:
            return 0.0
        return time.time() - self.task_start_time

    def publish_status(self, force=False):
        """Publish executor state as JSON."""
        now = time.time()
        if not force and now - self.last_status_time < 0.5:
            return

        self.last_status_time = now
        status = {
            'state': self.state,
            'busy': self.current_task is not None,
            'autonomous_enabled': self.autonomous_enabled,
            'plan_loaded': self.plan_loaded,
            'plan_path': self.plan_path,
            'task_id': None,
            'row_id': None,
            'tray_id': None,
            'segment_id': None,
            'task_type': None,
            'current_task_index': self.current_task_index,
            'total_tasks': len(self.mission_tasks),
            'retry_count': self.retry_count,
            'strafe_blocked': self.strafe_blocked_since is not None,
            'blocked_trays': list(self.blocked_trays.keys()),
        }
        if self.current_task is not None:
            status['task_id'] = self.current_task['task_id']
            status['row_id'] = self.current_task.get('row_id')
            status['tray_id'] = self.current_task.get('tray_id')
            status['segment_id'] = self.current_task.get('segment_id')
            status['task_type'] = self.current_task['type']

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_dashboard(self):
        """Publish a compact text dashboard for echoing in the terminal."""
        lines = ['--- GREENHOUSE SCAN DASHBOARD ---']
        mode = '[RUNNING]' if self.autonomous_enabled else '[PAUSED/TELEOP]'
        lines.append(f'STATUS: {mode}')
        lines.append(f'STATE: {self.state}')
        plan_source = self.plan_path if self.plan_loaded else 'topic'
        lines.append(f'PLAN: {plan_source}')

        if self.current_task is not None:
            lines.append(f"CURRENT: {self.current_task['task_id']}")
            if self.current_task.get('tray_id') is not None:
                lines.append(f"TRAY: {self.current_task['tray_id']}")

        if self.blocked_trays:
            lines.append(
                'TEMP BLOCKED TRAYS: '
                + ', '.join(sorted(self.blocked_trays.keys()))
            )

        lines.append('')
        if self.mission_tasks:
            for index, task in enumerate(self.mission_tasks):
                lines.append(self.dashboard_task_line(index, task))
        elif self.current_task is not None:
            lines.append(
                f"[ >> ] {self.current_task['task_id']} "
                f"({self.current_task['type']})"
            )
        else:
            lines.append('[    ] waiting for planner task')

        msg = String()
        msg.data = '\n'.join(lines)
        self.dashboard_pub.publish(msg)

    def dashboard_task_line(self, index, task):
        """Format one task for the dashboard."""
        task_id = task['task_id']
        if task_id in self.completed_task_ids:
            status = '[DONE]'
        elif self.current_task is not None and task_id == self.current_task[
            'task_id'
        ]:
            status = '[ >> ]' if self.autonomous_enabled else '[NEXT]'
        elif index == self.current_task_index:
            status = '[NEXT]'
        else:
            status = '[    ]'

        line = f"{status} {task_id} ({task['type']})"
        if task.get('tray_id') in self.blocked_trays:
            line += ' - tray blocked'
        if task_id in self.completed_task_ids:
            line += f" - {self.history[task_id]['duration']}"
        elif task_id in self.history:
            count = self.history[task_id].get('completed_count', 1)
            line += f' - runs={count}'
        return line

    def publish_markers(self):
        """Visualize the saved mission and current external task in RViz."""
        marker_array = MarkerArray()
        marker_array.markers.append(self.delete_all_marker())
        now = self.get_clock().now().to_msg()

        marker_id = 1
        for index, task in enumerate(self.mission_tasks):
            color = self.marker_color_for_task(index, task)
            marker_id = self.add_task_markers(
                marker_array,
                marker_id,
                task,
                color,
                now,
            )

        if not self.mission_tasks and self.current_task is not None:
            marker_id = self.add_task_markers(
                marker_array,
                marker_id,
                self.current_task,
                (0.0, 1.0, 0.0, 0.9),
                now,
            )

        self.marker_pub.publish(marker_array)

    def delete_all_marker(self):
        """Create a marker that clears stale mission markers."""
        marker = Marker()
        marker.action = Marker.DELETEALL
        return marker

    def marker_color_for_task(self, index, task):
        """Return RGBA marker color based on task progress."""
        task_id = task['task_id']
        if task.get('tray_id') in self.blocked_trays:
            return (1.0, 0.0, 0.0, 0.85)
        if task_id in self.completed_task_ids:
            return (0.3, 0.3, 0.3, 0.7)
        if self.current_task is not None and task_id == self.current_task[
            'task_id'
        ]:
            return (0.0, 1.0, 0.0, 0.95)
        if index == self.current_task_index:
            return (1.0, 0.8, 0.0, 0.9)
        return (1.0, 0.5, 0.0, 0.75)

    def add_task_markers(self, marker_array, marker_id, task, color, stamp):
        """Add pose, label, and scan-line markers for one task."""
        frame_id = task.get('frame_id', 'map')
        label_pose = self.primary_pose_for_task(task)
        marker_id = self.add_pose_marker(
            marker_array,
            marker_id,
            frame_id,
            stamp,
            label_pose,
            color,
            Marker.SPHERE,
        )

        if task['type'] == 'SCAN_ROW':
            marker_id = self.add_pose_marker(
                marker_array,
                marker_id,
                frame_id,
                stamp,
                task['scan_end_pose'],
                color,
                Marker.CUBE,
            )
            marker_id = self.add_scan_line_marker(
                marker_array,
                marker_id,
                frame_id,
                stamp,
                task,
                color,
            )

        marker_id = self.add_label_marker(
            marker_array,
            marker_id,
            frame_id,
            stamp,
            label_pose,
            task['task_id'],
        )
        return marker_id

    def add_pose_marker(
        self,
        marker_array,
        marker_id,
        frame_id,
        stamp,
        pose,
        color,
        marker_type,
    ):
        """Add a sphere or cube marker at a task pose."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'mainloop_tasks'
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker_array.markers.append(marker)
        return marker_id + 1

    def add_scan_line_marker(
        self,
        marker_array,
        marker_id,
        frame_id,
        stamp,
        task,
        color,
    ):
        """Draw the segment between approach and scan-end poses."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'mainloop_scan_lines'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.04
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.points = [
            Point(
                x=task['approach_pose'][0],
                y=task['approach_pose'][1],
                z=0.05,
            ),
            Point(
                x=task['scan_end_pose'][0],
                y=task['scan_end_pose'][1],
                z=0.05,
            ),
        ]
        marker_array.markers.append(marker)
        return marker_id + 1

    def add_label_marker(
        self,
        marker_array,
        marker_id,
        frame_id,
        stamp,
        pose,
        text,
    ):
        """Add a text label above the task pose."""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = 'mainloop_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = pose[0]
        marker.pose.position.y = pose[1]
        marker.pose.position.z = 0.42
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.text = text
        marker_array.markers.append(marker)
        return marker_id + 1

    @staticmethod
    def primary_pose_for_task(task):
        """Return the pose that represents a task start/target."""
        if task['type'] == 'SCAN_ROW':
            return task['approach_pose']
        if task['type'] == 'NAV_ONLY':
            return task['goal_pose']
        return task['scan_end_pose']

    def _call_perception(self, client, name):
        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f'Perception service {name} not available, skipping')
            return
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if future.done():
            self.get_logger().info(f'{name}: {future.result().message}')
        else:
            self.get_logger().warn(f'{name} call timed out')

    def force_stop_robot(self):
        """Publish zero velocity on both command and stop channels."""
        msg = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(msg)
            self.stop_pub.publish(msg)
            time.sleep(0.01)

    @staticmethod
    def clamp(value, minimum, maximum):
        """Clamp a numeric value."""
        return max(min(value, maximum), minimum)

    @staticmethod
    def get_yaw_from_pose(pose):
        """Extract planar yaw from a quaternion pose."""
        q = pose.orientation
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )


def main(args=None):
    """Run the merged mainloop node."""
    rclpy.init(args=args)
    node = MainLoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopping node...')
        node.force_stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

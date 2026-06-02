"""Mission executor for JSON row plans and planner-provided tasks."""

import json
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
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

        self.declare_parameter('strafe_speed', 0.2)
        self.declare_parameter('strafe_tolerance', 0.15)
        self.declare_parameter('yaw_gain', 0.5)
        self.declare_parameter('max_yaw_rate', 0.5)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('max_retries', 3)

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

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        stop_topic = self.get_parameter('cmd_vel_stop_topic').value
        idle_topic = self.get_parameter('cmd_vel_idle_topic').value
        task_topic = self.get_parameter('task_topic').value
        status_topic = self.get_parameter('status_topic').value
        result_topic = self.get_parameter('result_topic').value
        dashboard_topic = self.get_parameter('dashboard_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        nav_action_name = self.get_parameter('nav_action_name').value

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

        self.timer = self.create_timer(0.05, self.control_loop)
        self.publish_status(force=True)

    def expand_path(self, parameter_name):
        """Expand a string path parameter."""
        value = self.get_parameter(parameter_name).value
        return os.path.expanduser(str(value))

    def load_history(self):
        """Load completed task ids from disk."""
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
        """Persist a successfully completed task."""
        task_id = task['task_id']
        self.history[task_id] = {
            'completed_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'duration': f'{duration:.1f}s',
            'task_type': task['type'],
            'row_id': task.get('row_id'),
        }

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
        """Accept row-plan, task-list, and old mission-segment JSON formats."""
        if isinstance(raw_plan, list):
            raw_tasks = raw_plan
            return_home_pose = None
        elif isinstance(raw_plan, dict):
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

    def get_first_incomplete_task(self):
        """Return the first plan index that is not in mission history."""
        for index, task in enumerate(self.mission_tasks):
            if task['task_id'] not in self.history:
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

        task = {
            'task_id': str(task_id),
            'type': task_type,
            'row_id': row_id,
            'frame_id': raw_task.get('frame_id', 'map'),
        }

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
        """Check whether AMCL pose exists and is fresh enough."""
        if self.current_pose is None or self.last_pose_time is None:
            self.get_logger().warn(
                'Waiting for robot pose on /amcl_pose...',
                throttle_duration_sec=2.0,
            )
            return False

        if self.pose_timeout_sec <= 0.0:
            return True

        age = (
            self.get_clock().now() - self.last_pose_time
        ).nanoseconds / 1e9
        if age > self.pose_timeout_sec:
            self.get_logger().warn(
                f'Robot pose is stale ({age:.2f}s old)',
                throttle_duration_sec=2.0,
            )
            return False

        return True

    def has_pending_plan_task(self):
        """Return true when the saved plan still has incomplete tasks."""
        if not self.mission_tasks:
            return False

        for task in self.mission_tasks[self.current_task_index:]:
            if task['task_id'] not in self.history:
                return True
        return False

    def start_next_plan_task(self):
        """Load the next incomplete task from the saved plan."""
        while self.current_task_index < len(self.mission_tasks):
            task = self.mission_tasks[self.current_task_index]
            if task['task_id'] not in self.history:
                self.current_task = task
                self.current_task_is_external = False
                self.task_start_time = None
                self.retry_count = 0
                self.nav_phase = None
                self.state = 'TASK_READY'
                self.get_logger().info(
                    f"Selected plan task {task['task_id']} ({task['type']})"
                )
                self.publish_status(force=True)
                return
            self.current_task_index += 1

        self.handle_mission_complete()

    def handle_mission_complete(self):
        """Stop or restart after all saved-plan tasks are complete."""
        if self.loop_mission and self.mission_tasks:
            self.get_logger().info('Mission complete; restarting plan loop')
            self.history = {}
            if os.path.exists(self.history_path):
                os.remove(self.history_path)
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
            self.strafe_start_yaw = self.get_yaw_from_pose(self.current_pose)
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

        self.get_logger().info(
            f'Strafing: dist={distance:.2f}m, '
            f'local_err=({local_x:.2f}, {local_y:.2f}), '
            f'cmd=({twist.linear.x:.2f}, {twist.linear.y:.2f})',
            throttle_duration_sec=1.0,
        )
        self.cmd_vel_pub.publish(twist)

    def handle_task_failure(self, reason):
        """Retry a failed task until max_retries is reached."""
        self.force_stop_robot()
        self.nav_goal_handle = None
        self.nav_phase = None
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
            'task_type': None,
            'current_task_index': self.current_task_index,
            'total_tasks': len(self.mission_tasks),
            'retry_count': self.retry_count,
        }
        if self.current_task is not None:
            status['task_id'] = self.current_task['task_id']
            status['row_id'] = self.current_task.get('row_id')
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
        if task_id in self.history:
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
        if task_id in self.history:
            line += f" - {self.history[task_id]['duration']}"
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
        if task_id in self.history:
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

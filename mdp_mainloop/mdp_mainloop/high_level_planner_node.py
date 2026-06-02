"""Choose discrete row-scan tasks and dispatch them to the executor."""

import json
import math
import os
import time
from datetime import datetime

import rclpy
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_PLAN = {
    'rows': [
        {
            'id': 'row_a',
            'approach_pose': [1.439, -1.016, 2.506],
            'scan_end_pose': [2.146, 0.189, 2.600],
        },
        {
            'id': 'row_b',
            'approach_pose': [1.226, 0.798, -2.324],
            'scan_end_pose': [0.073, -0.709, -0.560],
        },
    ],
    'return_home_pose': [1.439, -1.016, 2.506],
}


class HighLevelPlannerNode(Node):
    """Own the discrete row set and choose which row should run next."""

    def __init__(self):
        super().__init__('mdp_high_level_planner_node')
        self.get_logger().info('MDP high-level planner started')

        self.declare_parameter('row_plan_path', '')
        self.declare_parameter(
            'generated_row_plan_path',
            '~/mdp_ws/generated_row_plan.json',
        )
        self.declare_parameter('require_generated_row_plan', False)
        self.declare_parameter('row_plan_file_poll_period', 1.0)
        self.declare_parameter('rows_json', '')
        self.declare_parameter('history_path', '~/mdp_ws/mission_history.json')
        self.declare_parameter('clear_history', False)
        self.declare_parameter('planner_input_topic', '/planner/row_scores')
        self.declare_parameter('row_plan_topic', '/planner/row_plan')
        self.declare_parameter('task_topic', '/planner/next_task')
        self.declare_parameter('executor_status_topic', '/mainloop/status')
        self.declare_parameter('executor_result_topic', '/mainloop/task_result')
        self.declare_parameter(
            'discrete_state_topic',
            '/planner/discrete_state',
        )
        self.declare_parameter('planner_status_topic', '/planner/status')
        self.declare_parameter('dashboard_topic', '/mission_dashboard')
        self.declare_parameter('marker_topic', '/mission_markers')
        self.declare_parameter('auto_dispatch', True)
        self.declare_parameter('return_home_after_rows', True)
        self.declare_parameter('skip_failed_rows', True)
        self.declare_parameter('dispatch_period', 1.0)

        self.row_plan_path = self.get_parameter('row_plan_path').value
        generated_plan_path = self.get_parameter(
            'generated_row_plan_path').value
        self.generated_row_plan_path = os.path.expanduser(generated_plan_path)
        self.require_generated_row_plan = self.get_parameter(
            'require_generated_row_plan').value
        self.row_plan_file_poll_period = self.get_parameter(
            'row_plan_file_poll_period').value
        self.rows_json = self.get_parameter('rows_json').value
        history_path = self.get_parameter('history_path').value
        self.history_path = os.path.expanduser(history_path)
        self.clear_history = self.get_parameter('clear_history').value
        self.auto_dispatch = self.get_parameter('auto_dispatch').value
        self.return_home_after_rows = self.get_parameter(
            'return_home_after_rows').value
        self.skip_failed_rows = self.get_parameter('skip_failed_rows').value
        self.dispatch_period = self.get_parameter('dispatch_period').value

        planner_input_topic = self.get_parameter(
            'planner_input_topic').value
        row_plan_topic = self.get_parameter('row_plan_topic').value
        task_topic = self.get_parameter('task_topic').value
        executor_status_topic = self.get_parameter(
            'executor_status_topic').value
        executor_result_topic = self.get_parameter(
            'executor_result_topic').value
        discrete_state_topic = self.get_parameter(
            'discrete_state_topic').value
        planner_status_topic = self.get_parameter(
            'planner_status_topic').value
        dashboard_topic = self.get_parameter('dashboard_topic').value
        marker_topic = self.get_parameter('marker_topic').value

        self.rows, self.return_home_pose = self.load_plan()
        self.row_by_id = {row['id']: row for row in self.rows}

        if self.clear_history and os.path.exists(self.history_path):
            os.remove(self.history_path)
            self.get_logger().info('Mission history cleared by parameter')

        self.history = {}
        self.completed_ids = set()
        self.failed_ids = set()
        self.load_history()

        self.row_scores = {}
        self.requested_row_id = None
        self.blocked_rows = set()
        self.autonomous_enabled = False
        self.executor_busy = False
        self.executor_state = 'UNKNOWN'
        self.executor_task_id = None
        self.pending_task = None
        self.active_task_id = None
        self.active_row_id = None
        self.last_task_publish_time = 0.0
        self.last_plan_file_check = 0.0
        self.plan_file_mtime = None
        self.discrete_state = {
            'state': 'IDLE',
            'reason': 'waiting_for_autonomy',
        }

        self.task_pub = self.create_publisher(String, task_topic, 10)
        self.discrete_state_pub = self.create_publisher(
            String,
            discrete_state_topic,
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            planner_status_topic,
            10,
        )
        self.dashboard_pub = self.create_publisher(
            String,
            dashboard_topic,
            10,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            10,
        )

        self.enable_sub = self.create_subscription(
            Bool,
            '/autonomous_enabled',
            self.enable_callback,
            10,
        )
        self.planner_input_sub = self.create_subscription(
            String,
            planner_input_topic,
            self.planner_input_callback,
            10,
        )
        latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.row_plan_sub = self.create_subscription(
            String,
            row_plan_topic,
            self.row_plan_callback,
            latched_qos,
        )
        self.executor_status_sub = self.create_subscription(
            String,
            executor_status_topic,
            self.executor_status_callback,
            10,
        )
        self.executor_result_sub = self.create_subscription(
            String,
            executor_result_topic,
            self.executor_result_callback,
            10,
        )
        self.click_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.click_callback,
            10,
        )

        self.timer = self.create_timer(0.5, self.planner_loop)

    def row_plan_callback(self, msg):
        """Replace the discrete row set from a published row plan."""
        try:
            plan = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid row plan: {exc}')
            return

        self.apply_row_plan(plan, 'topic')

    def load_plan(self):
        """Load rows from a JSON file, a JSON parameter, or defaults."""
        plan = DEFAULT_PLAN

        generated_plan = self.load_plan_file(self.generated_row_plan_path)
        if generated_plan is not None:
            plan = generated_plan
        elif self.require_generated_row_plan:
            self.get_logger().error(
                'Required generated row plan is missing or invalid: '
                f'{self.generated_row_plan_path}'
            )
            return [], None

        if self.row_plan_path:
            seed_plan = self.load_plan_file(self.row_plan_path)
            if (
                not self.require_generated_row_plan
                and generated_plan is None
                and seed_plan is not None
            ):
                plan = seed_plan

        if self.rows_json:
            try:
                loaded = json.loads(self.rows_json)
                plan = loaded if isinstance(loaded, dict) else {'rows': loaded}
                self.get_logger().info('Loaded rows from rows_json parameter')
            except json.JSONDecodeError as exc:
                self.get_logger().warn(
                    f'Invalid rows_json parameter: {exc}. Using plan file.'
                )

        rows, return_home_pose = self.parse_plan(plan)

        if not rows:
            if self.require_generated_row_plan:
                self.get_logger().error(
                    'Generated row plan has no valid rows'
                )
                return rows, return_home_pose

            self.get_logger().error('No valid rows in plan; using defaults')
            rows, return_home_pose = self.parse_plan(DEFAULT_PLAN)

        return rows, return_home_pose

    def load_plan_file(self, path):
        """Read one row-plan JSON file."""
        if not path:
            return None

        path = os.path.expanduser(path)
        if not os.path.exists(path):
            return None

        try:
            with open(path, 'r') as file:
                plan = json.load(file)
        except OSError as exc:
            self.get_logger().warn(
                f'Could not read row plan {path}: {exc}'
            )
            return None
        except json.JSONDecodeError as exc:
            self.get_logger().warn(
                f'Invalid row plan JSON {path}: {exc}'
            )
            return None

        self.get_logger().info(f'Loaded row plan from {path}')
        return plan

    def maybe_reload_row_plan_file(self):
        """Poll the generated JSON file for row-plan changes."""
        if self.row_plan_file_poll_period <= 0:
            return

        now = time.time()
        if now - self.last_plan_file_check < self.row_plan_file_poll_period:
            return

        self.last_plan_file_check = now
        path = self.generated_row_plan_path
        if not path or not os.path.exists(path):
            return

        try:
            mtime = os.path.getmtime(path)
        except OSError as exc:
            self.get_logger().warn(
                f'Could not stat generated row plan {path}: {exc}'
            )
            return

        if self.plan_file_mtime == mtime:
            return

        plan = self.load_plan_file(path)
        if plan is None:
            return

        self.plan_file_mtime = mtime
        self.apply_row_plan(plan, 'file')

    def apply_row_plan(self, plan, source):
        """Apply a row-plan object to the planner state."""
        rows, return_home_pose = self.parse_plan(plan)

        self.rows = rows
        self.return_home_pose = return_home_pose
        self.row_by_id = {row['id']: row for row in self.rows}
        self.requested_row_id = self.keep_known_row(self.requested_row_id)
        self.blocked_rows = {
            row_id for row_id in self.blocked_rows
            if row_id in self.row_by_id
        }
        self.row_scores = {
            row_id: score for row_id, score in self.row_scores.items()
            if row_id in self.row_by_id
        }
        if self.pending_task is not None:
            pending_row = self.pending_task.get('row_id')
            if pending_row is not None and pending_row not in self.row_by_id:
                self.pending_task = None
        self.get_logger().info(
            f'Reloaded row plan from {source} with {len(self.rows)} rows'
        )

    def parse_plan(self, plan):
        """Validate a whole row-plan object."""
        if isinstance(plan, list):
            plan = {'rows': plan}
        if not isinstance(plan, dict):
            return [], None

        rows = []
        for index, raw_row in enumerate(plan.get('rows', [])):
            row = self.parse_row(raw_row, index)
            if row is not None:
                rows.append(row)

        return_home_pose = self.parse_pose(plan.get('return_home_pose'))
        return rows, return_home_pose

    def parse_row(self, raw_row, index):
        """Validate one row definition from the plan."""
        if not isinstance(raw_row, dict):
            self.get_logger().warn(f'Ignoring row {index}; not an object')
            return None

        row_id = raw_row.get('id') or f'row_{index + 1}'
        approach_pose = self.parse_pose(raw_row.get('approach_pose'))
        scan_end_pose = self.parse_pose(
            raw_row.get('scan_end_pose') or raw_row.get('goal_pose')
        )

        if approach_pose is None or scan_end_pose is None:
            self.get_logger().warn(
                f'Ignoring row {row_id}; missing valid approach/goal poses'
            )
            return None

        return {
            'id': str(row_id),
            'approach_pose': approach_pose,
            'scan_end_pose': scan_end_pose,
        }

    @staticmethod
    def parse_pose(raw_pose):
        """Return [x, y, yaw] from a row-plan pose."""
        if not isinstance(raw_pose, (list, tuple)) or len(raw_pose) < 2:
            return None

        try:
            x = float(raw_pose[0])
            y = float(raw_pose[1])
            yaw = None if len(raw_pose) < 3 else float(raw_pose[2])
        except (TypeError, ValueError):
            return None

        return [x, y, yaw]

    def load_history(self):
        """Load completed task IDs from disk."""
        if not os.path.exists(self.history_path):
            return

        try:
            with open(self.history_path, 'r') as file:
                self.history = json.load(file)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(
                f'Could not load mission history: {exc}'
            )
            self.history = {}
            return

        for task_id, record in self.history.items():
            if not isinstance(record, dict):
                continue
            if record.get('success', True):
                self.completed_ids.add(task_id)

    def save_history(self):
        """Persist completed task IDs."""
        history_dir = os.path.dirname(self.history_path)
        if history_dir:
            os.makedirs(history_dir, exist_ok=True)

        with open(self.history_path, 'w') as file:
            json.dump(self.history, file, indent=4)

    def enable_callback(self, msg):
        """Track autonomy so the planner only dispatches while enabled."""
        self.autonomous_enabled = msg.data
        if not self.autonomous_enabled:
            self.discrete_state = {
                'state': 'PAUSED',
                'reason': 'autonomous_disabled',
            }

    def planner_input_callback(self, msg):
        """Update row scores or explicit row requests from a JSON topic."""
        text = msg.data.strip()
        if text in self.row_by_id:
            self.requested_row_id = text
            self.get_logger().info(f'Requested row from topic: {text}')
            return

        try:
            data = json.loads(text)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid planner input: {exc}')
            return

        if not isinstance(data, dict):
            self.get_logger().warn('Planner input must be a JSON object')
            return

        selected_row = data.get('selected_row') or data.get('requested_row')
        if selected_row is not None:
            selected_row = str(selected_row)
            if selected_row in self.row_by_id:
                self.requested_row_id = selected_row
                self.get_logger().info(
                    f'Requested row from topic: {selected_row}'
                )

        if data.get('force_rescan') and self.requested_row_id:
            self.clear_completion(self.requested_row_id)

        scores = data.get('row_scores') or data.get('scores')
        if scores is None:
            scores = self.extract_numeric_score_map(data)
        self.update_scores(scores)

        blocked_rows = data.get('blocked_rows')
        if isinstance(blocked_rows, list):
            self.blocked_rows = {
                str(row_id)
                for row_id in blocked_rows
                if str(row_id) in self.row_by_id
            }

    def extract_numeric_score_map(self, data):
        """Treat a flat JSON object as row_id -> score when possible."""
        scores = {}
        for row_id, value in data.items():
            if row_id not in self.row_by_id:
                continue
            try:
                scores[row_id] = float(value)
            except (TypeError, ValueError):
                continue
        return scores

    def update_scores(self, scores):
        """Store the latest numeric row scores."""
        if not isinstance(scores, dict):
            return

        for row_id, score in scores.items():
            row_id = str(row_id)
            if row_id not in self.row_by_id:
                continue
            try:
                self.row_scores[row_id] = float(score)
            except (TypeError, ValueError):
                self.get_logger().warn(
                    f'Ignoring non-numeric score for {row_id}'
                )

    def executor_status_callback(self, msg):
        """Track whether the executor is available for a new task."""
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        self.executor_busy = bool(status.get('busy', False))
        self.executor_state = status.get('state', 'UNKNOWN')
        self.executor_task_id = status.get('task_id')
        self.active_row_id = status.get('row_id')

        if self.executor_busy and self.pending_task is not None:
            if self.pending_task['task_id'] == self.executor_task_id:
                self.active_task_id = self.executor_task_id
                self.pending_task = None

    def executor_result_callback(self, msg):
        """Record executor results and free the planner to choose again."""
        try:
            result = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid task result: {exc}')
            return

        task_id = result.get('task_id')
        row_id = result.get('row_id')
        success = bool(result.get('success', False))
        completion_id = row_id or task_id

        if success:
            self.failed_ids.discard(completion_id)
            self.completed_ids.add(completion_id)
            self.history[completion_id] = {
                'completed_at': datetime.now().strftime(
                    '%Y-%m-%d %H:%M:%S'
                ),
                'duration': result.get('duration', 0.0),
                'task_id': task_id,
                'success': True,
            }
            self.save_history()
            if row_id == self.requested_row_id:
                self.requested_row_id = None
            self.get_logger().info(f'Completed {completion_id}')
        else:
            self.failed_ids.add(completion_id)
            self.get_logger().warn(
                f"Task {completion_id} failed: "
                f"{result.get('reason', 'unknown')}"
            )

        self.pending_task = None
        self.active_task_id = None
        self.active_row_id = None

    def click_callback(self, msg):
        """Allow RViz clicked points to select a row to scan."""
        click_x = msg.point.x
        click_y = msg.point.y
        best_dist = float('inf')
        best_row_id = None

        for row in self.rows:
            dist = min(
                self.distance_to_pose(click_x, click_y, row['approach_pose']),
                self.distance_to_pose(click_x, click_y, row['scan_end_pose']),
            )
            if dist < best_dist:
                best_dist = dist
                best_row_id = row['id']

        if best_row_id is not None and best_dist < 2.0:
            self.requested_row_id = best_row_id
            self.clear_completion(best_row_id)
            self.get_logger().info(
                f'Manual row selection from RViz: {best_row_id}'
            )

    def planner_loop(self):
        """Publish planner state and dispatch work when possible."""
        self.maybe_reload_row_plan_file()
        self.publish_markers()
        self.publish_dashboard()
        self.publish_planner_status()
        self.publish_discrete_state()

        if not self.auto_dispatch:
            self.discrete_state = {
                'state': 'IDLE',
                'reason': 'auto_dispatch_disabled',
            }
            return

        if not self.autonomous_enabled:
            self.discrete_state = {
                'state': 'PAUSED',
                'reason': 'autonomous_disabled',
            }
            return

        if self.executor_busy:
            self.discrete_state = {
                'state': 'EXECUTOR_BUSY',
                'task_id': self.executor_task_id,
                'row_id': self.active_row_id,
                'executor_state': self.executor_state,
            }
            return

        if self.pending_task is not None:
            self.publish_pending_task()
            return

        task, decision = self.choose_next_task()
        self.discrete_state = decision
        if task is None:
            return

        self.pending_task = task
        self.publish_pending_task(force=True)

    def choose_next_task(self):
        """Return the next executor task and a discrete-state message."""
        if not self.rows:
            return None, {
                'state': 'NO_ROW_PLAN',
                'reason': 'row_plan_missing_empty_or_invalid',
                'plan_path': self.generated_row_plan_path,
            }

        row, reason, score = self.choose_next_row()
        if row is not None:
            task_id = f"scan_{row['id']}"
            task = {
                'task_id': task_id,
                'type': 'SCAN_ROW',
                'row_id': row['id'],
                'frame_id': 'map',
                'approach_pose': row['approach_pose'],
                'scan_end_pose': row['scan_end_pose'],
            }
            decision = {
                'state': 'ROW_SELECTED',
                'row_id': row['id'],
                'task_id': task_id,
                'reason': reason,
            }
            if score is not None:
                decision['score'] = score
            return task, decision

        if self.should_return_home():
            task = {
                'task_id': 'return_home',
                'type': 'NAV_ONLY',
                'row_id': None,
                'frame_id': 'map',
                'goal_pose': self.return_home_pose,
            }
            decision = {
                'state': 'RETURN_HOME_SELECTED',
                'task_id': 'return_home',
                'reason': 'all_rows_completed',
            }
            return task, decision

        if self.has_unavailable_rows():
            return None, {
                'state': 'NO_AVAILABLE_ROWS',
                'reason': 'remaining_rows_blocked_or_failed',
                'failed_rows': sorted(self.failed_ids),
                'blocked_rows': sorted(self.blocked_rows),
            }

        return None, {
            'state': 'MISSION_COMPLETE',
            'reason': 'all_tasks_completed',
        }

    def choose_next_row(self):
        """Choose a row using request, score, then plan order."""
        available_rows = self.get_available_rows()
        if not available_rows:
            return None, None, None

        available_ids = {row['id'] for row in available_rows}
        if self.requested_row_id in available_ids:
            row = self.row_by_id[self.requested_row_id]
            return row, 'explicit_request', self.row_scores.get(row['id'])

        scored_rows = [
            row for row in available_rows
            if row['id'] in self.row_scores
        ]
        if scored_rows:
            row = max(scored_rows, key=lambda item: self.row_scores[item['id']])
            return row, 'highest_score', self.row_scores[row['id']]

        return available_rows[0], 'plan_order', None

    def get_available_rows(self):
        """Return rows that are not completed, blocked, or failed."""
        rows = []
        for row in self.rows:
            row_id = row['id']
            if row_id in self.completed_ids:
                continue
            if row_id in self.blocked_rows:
                continue
            if self.skip_failed_rows and row_id in self.failed_ids:
                continue
            rows.append(row)
        return rows

    def should_return_home(self):
        """Return true when the planner should dispatch the home task."""
        if not self.return_home_after_rows:
            return False
        if self.return_home_pose is None:
            return False
        all_rows_done = all(row['id'] in self.completed_ids
                            for row in self.rows)
        return all_rows_done and 'return_home' not in self.completed_ids

    def has_unavailable_rows(self):
        """Return true when rows remain but none are dispatchable."""
        all_row_ids = {row['id'] for row in self.rows}
        return not all_row_ids.issubset(self.completed_ids)

    def publish_pending_task(self, force=False):
        """Publish the pending task until the executor acknowledges it."""
        now = time.time()
        if not force and now - self.last_task_publish_time < self.dispatch_period:
            return

        msg = String()
        msg.data = json.dumps(self.pending_task)
        self.task_pub.publish(msg)
        self.last_task_publish_time = now
        self.discrete_state = {
            'state': 'DISPATCHING_TASK',
            'task_id': self.pending_task['task_id'],
            'row_id': self.pending_task.get('row_id'),
        }
        self.get_logger().info(
            f"Dispatching task {self.pending_task['task_id']}",
            throttle_duration_sec=2.0,
        )

    def publish_discrete_state(self):
        """Publish the current discrete planner state."""
        msg = String()
        msg.data = json.dumps(self.discrete_state)
        self.discrete_state_pub.publish(msg)

    def publish_planner_status(self):
        """Publish structured planner status as JSON."""
        status = {
            'state': self.discrete_state.get('state'),
            'autonomous_enabled': self.autonomous_enabled,
            'executor_busy': self.executor_busy,
            'executor_state': self.executor_state,
            'executor_task_id': self.executor_task_id,
            'completed_ids': sorted(self.completed_ids),
            'failed_ids': sorted(self.failed_ids),
            'blocked_rows': sorted(self.blocked_rows),
            'requested_row_id': self.requested_row_id,
            'row_scores': self.row_scores,
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def publish_dashboard(self):
        """Publish a human-readable mission dashboard."""
        lines = ['--- GREENHOUSE HIGH-LEVEL PLANNER ---']
        status = '[RUNNING]' if self.autonomous_enabled else '[PAUSED]'
        lines.append(f'STATUS: {status}')
        lines.append(f"PLANNER: {self.discrete_state.get('state')}")
        lines.append(f'EXECUTOR: {self.executor_state}')
        lines.append('')

        for row in self.rows:
            row_id = row['id']
            marker = '[    ]'
            if row_id in self.completed_ids:
                marker = '[DONE]'
            elif row_id in self.failed_ids:
                marker = '[FAIL]'
            elif row_id == self.active_row_id:
                marker = '[ >> ]'
            elif row_id == self.requested_row_id:
                marker = '[NEXT]'
            score = self.row_scores.get(row_id)
            suffix = '' if score is None else f' score={score:.2f}'
            lines.append(f'{marker} {row_id}{suffix}')

        if self.return_home_after_rows and self.return_home_pose is not None:
            home_marker = '[DONE]' if 'return_home' in self.completed_ids else '[    ]'
            if self.executor_task_id == 'return_home':
                home_marker = '[ >> ]'
            lines.append(f'{home_marker} return_home')

        msg = String()
        msg.data = '\n'.join(lines)
        self.dashboard_pub.publish(msg)

    def publish_markers(self):
        """Publish RViz markers for all planned rows."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for index, row in enumerate(self.rows):
            color = self.row_color(row['id'])
            approach = row['approach_pose']
            scan_end = row['scan_end_pose']

            marker_array.markers.append(
                self.make_pose_marker(
                    'row_approach',
                    index * 10,
                    approach,
                    Marker.SPHERE,
                    color,
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_pose_marker(
                    'row_scan_end',
                    index * 10 + 1,
                    scan_end,
                    Marker.CUBE,
                    color,
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_line_marker(
                    'row_scan_line',
                    index * 10 + 2,
                    approach,
                    scan_end,
                    color,
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_label_marker(
                    'row_labels',
                    index * 10 + 3,
                    row,
                    color,
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_arrow_marker(
                    'row_approach_heading',
                    index * 10 + 4,
                    approach,
                    color,
                    stamp,
                )
            )

        self.marker_pub.publish(marker_array)

    def row_color(self, row_id):
        """Return marker color for a row state."""
        if row_id in self.completed_ids:
            return 0.3, 0.3, 0.3, 0.8
        if row_id in self.failed_ids:
            return 1.0, 0.0, 0.0, 0.8
        if row_id == self.active_row_id:
            return 0.0, 1.0, 0.0, 1.0
        if row_id == self.requested_row_id:
            return 0.2, 0.8, 1.0, 1.0
        return 1.0, 0.5, 0.0, 0.8

    @staticmethod
    def apply_marker_color(marker, color):
        """Apply an RGBA tuple to a marker."""
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    def make_pose_marker(self, namespace, marker_id, pose, marker_type,
                         color, stamp):
        """Create a sphere or cube marker at a pose."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = 0.05
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        self.apply_marker_color(marker, color)
        return marker

    def make_arrow_marker(self, namespace, marker_id, pose, color, stamp):
        """Create a heading arrow marker."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = 0.12
        yaw = pose[2] if pose[2] is not None else 0.0
        marker.pose.orientation.z = math.sin(yaw / 2.0)
        marker.pose.orientation.w = math.cos(yaw / 2.0)
        marker.scale.x = 0.35
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        self.apply_marker_color(marker, color)
        return marker

    def make_line_marker(self, namespace, marker_id, start_pose, end_pose,
                         color, stamp):
        """Create a line marker for the strafe path."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.04
        marker.points = [
            Point(x=float(start_pose[0]), y=float(start_pose[1]), z=0.05),
            Point(x=float(end_pose[0]), y=float(end_pose[1]), z=0.05),
        ]
        self.apply_marker_color(marker, color)
        return marker

    def make_label_marker(self, namespace, marker_id, row, color, stamp):
        """Create a text label marker for a row."""
        pose = row['approach_pose']
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = 0.45
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.16
        marker.text = row['id']
        self.apply_marker_color(marker, color)
        return marker

    def clear_completion(self, completion_id):
        """Remove a row from history so it can be scanned again."""
        self.completed_ids.discard(completion_id)
        self.failed_ids.discard(completion_id)
        if completion_id in self.history:
            del self.history[completion_id]
            self.save_history()

    def keep_known_row(self, row_id):
        """Return the row ID only if it still exists in the current plan."""
        if row_id in self.row_by_id:
            return row_id
        return None

    @staticmethod
    def distance_to_pose(x, y, pose):
        """Return planar distance from a point to a pose."""
        return math.sqrt((x - pose[0]) ** 2 + (y - pose[1]) ** 2)


def main(args=None):
    """Run the high-level planner node."""
    rclpy.init(args=args)
    node = HighLevelPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

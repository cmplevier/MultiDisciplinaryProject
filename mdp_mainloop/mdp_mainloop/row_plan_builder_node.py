"""Build and publish row plans from dynamic row definitions."""

import json
import math
import os
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


DEFAULT_PLAN = {
    # The builder keeps the same shape on disk and on /planner/row_plan.
    'trays': [],
    'rows': [],
    'return_home_pose': None,
}


class RowPlanBuilderNode(Node):
    """Maintain a generated row-plan JSON file and publish it."""

    def __init__(self):
        super().__init__('mdp_row_plan_builder_node')
        self.get_logger().info('MDP row plan builder started')

        # File inputs/outputs. plan_path is the generated mission file.
        # seed_plan_path is only used as a fallback template.
        self.declare_parameter(
            'plan_path',
            '~/mdp_ws/generated_row_plan.json',
        )
        self.declare_parameter('seed_plan_path', '')
        self.declare_parameter('clear_on_start', False)

        # Topic API for editing the plan.
        self.declare_parameter('set_row_topic', '/row_plan/set_row')
        self.declare_parameter('set_tray_topic', '/row_plan/set_tray')
        self.declare_parameter('clear_plan_topic', '/row_plan/clear')
        self.declare_parameter('capture_mode', 'tray')
        self.declare_parameter(
            'approach_pose_topic',
            '/row_plan/approach_pose',
        )
        self.declare_parameter(
            'scan_end_pose_topic',
            '/row_plan/scan_end_pose',
        )
        self.declare_parameter('row_id_topic', '/row_plan/row_id')
        self.declare_parameter('tray_id_topic', '/row_plan/tray_id')
        self.declare_parameter('plan_topic', '/planner/row_plan')
        self.declare_parameter('status_topic', '/row_plan/status')
        self.declare_parameter('marker_topic', '/row_plan/markers')
        self.declare_parameter('publish_period', 1.0)

        self.plan_path = os.path.expanduser(
            self.get_parameter('plan_path').value
        )
        self.seed_plan_path = os.path.expanduser(
            self.get_parameter('seed_plan_path').value
        )
        self.clear_on_start = self.get_parameter('clear_on_start').value
        set_row_topic = self.get_parameter('set_row_topic').value
        set_tray_topic = self.get_parameter('set_tray_topic').value
        clear_plan_topic = self.get_parameter('clear_plan_topic').value
        self.capture_mode = str(
            self.get_parameter('capture_mode').value
        ).lower()
        approach_pose_topic = self.get_parameter(
            'approach_pose_topic').value
        scan_end_pose_topic = self.get_parameter(
            'scan_end_pose_topic').value
        row_id_topic = self.get_parameter('row_id_topic').value
        tray_id_topic = self.get_parameter('tray_id_topic').value
        plan_topic = self.get_parameter('plan_topic').value
        status_topic = self.get_parameter('status_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        self.publish_period = self.get_parameter('publish_period').value

        if self.clear_on_start:
            self.plan = self.empty_plan()
            self.get_logger().info('Starting with an empty generated plan')
        else:
            self.plan = self.load_initial_plan()
        self.last_publish_time = 0.0

        # RViz row capture is a two-step process:
        # first store the approach pose, then commit the row on scan-end pose.
        self.pending_approach_pose = None
        self.pending_row_id = None
        self.pending_tray_id = None

        # Latch the full plan so a planner started later receives it
        # immediately without waiting for the next periodic publish.
        latched_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=1,
        )
        self.plan_pub = self.create_publisher(
            String,
            plan_topic,
            latched_qos,
        )
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.marker_pub = self.create_publisher(
            MarkerArray,
            marker_topic,
            10,
        )

        # JSON command path: useful for scripts or terminal commands.
        self.set_row_sub = self.create_subscription(
            String,
            set_row_topic,
            self.set_row_callback,
            10,
        )
        self.set_tray_sub = self.create_subscription(
            String,
            set_tray_topic,
            self.set_tray_callback,
            10,
        )
        self.clear_sub = self.create_subscription(
            Bool,
            clear_plan_topic,
            self.clear_plan_callback,
            10,
        )

        # RViz path: set row ID, then use two pose-arrow tools.
        self.approach_pose_sub = self.create_subscription(
            PoseStamped,
            approach_pose_topic,
            self.approach_pose_callback,
            10,
        )
        self.scan_end_pose_sub = self.create_subscription(
            PoseStamped,
            scan_end_pose_topic,
            self.scan_end_pose_callback,
            10,
        )
        self.row_id_sub = self.create_subscription(
            String,
            row_id_topic,
            self.row_id_callback,
            10,
        )
        self.tray_id_sub = self.create_subscription(
            String,
            tray_id_topic,
            self.tray_id_callback,
            10,
        )

        # Periodic publication keeps status and markers visible in RViz.
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.save_plan()
        self.publish_plan(force=True)

    def load_initial_plan(self):
        """Load generated plan first, then seed file, then an empty plan."""
        # Prefer continuing from the generated file so authoring sessions can
        # be resumed. If it does not exist, fall back to the seed template.
        for path in [self.plan_path, self.seed_plan_path]:
            if not path:
                continue
            plan = self.load_plan_file(path)
            if plan is not None:
                return plan

        return self.empty_plan()

    @staticmethod
    def empty_plan():
        """Return a fresh empty plan dictionary."""
        return {
            'trays': [],
            'rows': [],
            'return_home_pose': None,
        }

    def load_plan_file(self, path):
        """Load and validate a row-plan JSON file."""
        if not os.path.exists(path):
            return None

        try:
            with open(path, 'r') as file:
                raw_plan = json.load(file)
        except (OSError, json.JSONDecodeError) as exc:
            self.get_logger().warn(f'Could not load row plan {path}: {exc}')
            return None

        plan = self.parse_plan(raw_plan)
        if plan is None:
            self.get_logger().warn(f'Ignoring invalid row plan {path}')
            return None

        self.get_logger().info(f'Loaded row plan from {path}')
        return plan

    def set_row_callback(self, msg):
        """Add, replace, delete, or bulk-load rows/trays from JSON."""
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid row command: {exc}')
            return

        if isinstance(command, list):
            command = {'rows': command}
        if not isinstance(command, dict):
            self.get_logger().warn('Row command must be a JSON object/list')
            return

        self.apply_plan_command(command)

    def set_tray_callback(self, msg):
        """Add, replace, delete, or bulk-load trays from JSON."""
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'Ignoring invalid tray command: {exc}')
            return

        if isinstance(command, list):
            command = {'trays': command}
        elif isinstance(command, dict) and 'waypoints' in command:
            command = {'tray': command}
        if not isinstance(command, dict):
            self.get_logger().warn('Tray command must be a JSON object/list')
            return

        self.apply_plan_command(command)

    def apply_plan_command(self, command):
        """Apply one JSON edit command to rows, trays, and return-home pose."""
        if command.get('clear'):
            self.plan = self.empty_plan()

        if 'return_home_pose' in command:
            pose = self.parse_pose(command.get('return_home_pose'))
            if pose is None and command.get('return_home_pose') is not None:
                self.get_logger().warn('Ignoring invalid return_home_pose')
            else:
                self.plan['return_home_pose'] = pose

        deleted = command.get('delete_row')
        if deleted is not None:
            self.delete_row(str(deleted))

        deleted_tray = command.get('delete_tray')
        if deleted_tray is not None:
            self.delete_tray(str(deleted_tray))

        rows = command.get('rows')
        if rows is not None:
            self.replace_rows(rows)
        elif 'approach_pose' in command and (
            'scan_end_pose' in command or 'goal_pose' in command
        ):
            self.upsert_row(command)

        trays = command.get('trays')
        if trays is not None:
            self.replace_trays(trays)
        elif command.get('tray') is not None:
            self.upsert_tray(command['tray'])
        elif 'waypoints' in command:
            self.upsert_tray(command)

        self.save_plan()
        self.publish_plan(force=True)

    def row_id_callback(self, msg):
        """Set the row ID used by the next PoseStamped capture pair."""
        row_id = msg.data.strip()
        if row_id:
            self.pending_row_id = row_id
            self.get_logger().info(f'Next captured row ID: {row_id}')

    def tray_id_callback(self, msg):
        """Set the tray ID used by the next two PoseStamped capture pairs."""
        tray_id = msg.data.strip()
        if tray_id:
            self.pending_tray_id = tray_id
            self.get_logger().info(f'Next captured tray ID: {tray_id}')

    def approach_pose_callback(self, msg):
        """Store the next approach pose for PoseStamped row capture."""
        # This only stores half of a row. The row is not written until the
        # matching scan-end pose arrives.
        self.pending_approach_pose = self.pose_stamped_to_list(msg)
        self.get_logger().info(
            f'Received start pose for {self.next_pending_capture_label()}'
        )

    def scan_end_pose_callback(self, msg):
        """Commit a row/tray segment after receiving its scan-end pose."""
        if self.pending_approach_pose is None:
            self.get_logger().warn(
                'Received scan_end_pose before approach_pose; ignoring'
            )
            return

        segment = {
            'approach_pose': self.pending_approach_pose,
            'scan_end_pose': self.pose_stamped_to_list(msg),
        }
        if self.capture_mode == 'row':
            segment['id'] = self.next_capture_row_id()
            self.upsert_row(segment)
            self.pending_row_id = None
        else:
            self.capture_tray_segment(segment)

        self.pending_approach_pose = None
        self.save_plan()
        self.publish_plan(force=True)

    def next_capture_row_id(self):
        """Return the row ID for the next PoseStamped capture."""
        # If the user did not publish /row_plan/row_id, auto-name rows in
        # insertion order. This keeps RViz-only authoring usable.
        if self.pending_row_id:
            return self.pending_row_id
        return f"row_{len(self.plan.get('rows', [])) + 1}"

    def next_capture_tray_id(self):
        """Return the tray ID for the next PoseStamped capture."""
        if self.pending_tray_id:
            return self.pending_tray_id

        for tray in self.plan.get('trays', []):
            if self.next_missing_tray_segment(tray) is not None:
                return tray['id']

        return f"tray_{len(self.plan.get('trays', [])) + 1}"

    def capture_tray_segment(self, segment):
        """Write one captured pose pair into A/B or C/D of a tray."""
        tray_id = self.next_capture_tray_id()
        tray = self.get_or_create_tray(tray_id)
        waypoint_pair = self.next_missing_tray_segment(tray)
        if waypoint_pair is None:
            self.get_logger().warn(
                f'Tray {tray_id} already has A/B/C/D; ignoring capture'
            )
            return

        start_key, end_key = waypoint_pair
        tray.setdefault('waypoints', {})
        tray['waypoints'][start_key] = segment['approach_pose']
        tray['waypoints'][end_key] = segment['scan_end_pose']
        self.get_logger().info(
            f'Captured tray {tray_id} segment {start_key}->{end_key}'
        )

        if self.next_missing_tray_segment(tray) is None:
            self.get_logger().info(f'Tray {tray_id} now has A/B/C/D')
            if self.pending_tray_id == tray_id:
                self.pending_tray_id = None

    def get_or_create_tray(self, tray_id):
        """Return an existing tray or append a new one."""
        tray = self.find_tray(tray_id)
        if tray is not None:
            return tray

        tray = {'id': tray_id, 'waypoints': {}}
        self.plan.setdefault('trays', []).append(tray)
        self.get_logger().info(f'Added tray {tray_id}')
        return tray

    def find_tray(self, tray_id):
        """Return an existing tray by ID, or None."""
        for tray in self.plan.get('trays', []):
            if tray['id'] == tray_id:
                return tray
        return None

    @staticmethod
    def next_missing_tray_segment(tray):
        """Return the next missing waypoint pair for a tray."""
        waypoints = tray.get('waypoints', {})
        if not all(name in waypoints for name in ['A', 'B']):
            return ('A', 'B')
        if not all(name in waypoints for name in ['C', 'D']):
            return ('C', 'D')
        return None

    def clear_plan_callback(self, msg):
        """Clear all generated rows when requested."""
        if not msg.data:
            return

        self.plan = self.empty_plan()
        self.pending_approach_pose = None
        self.pending_row_id = None
        self.pending_tray_id = None
        self.save_plan()
        self.publish_plan(force=True)
        self.get_logger().info('Generated row plan cleared')

    def replace_rows(self, rows):
        """Replace all rows with a validated row list."""
        if not isinstance(rows, list):
            self.get_logger().warn('rows must be a list')
            return

        validated_rows = []
        for index, raw_row in enumerate(rows):
            row = self.parse_row(raw_row, index)
            if row is not None:
                validated_rows.append(row)

        self.plan['rows'] = validated_rows
        self.get_logger().info(
            f'Replaced generated plan with {len(validated_rows)} rows'
        )

    def replace_trays(self, trays):
        """Replace all trays with a validated tray list."""
        if not isinstance(trays, list):
            self.get_logger().warn('trays must be a list')
            return

        validated_trays = []
        for index, raw_tray in enumerate(trays):
            tray = self.parse_tray(raw_tray, index)
            if tray is not None:
                validated_trays.append(tray)

        self.plan['trays'] = validated_trays
        self.get_logger().info(
            f'Replaced generated plan with {len(validated_trays)} trays'
        )

    def upsert_row(self, raw_row):
        """Add or replace one row definition."""
        row = self.parse_row(raw_row, len(self.plan['rows']))
        if row is None:
            return

        # Reusing an existing row ID updates that row instead of appending a
        # duplicate. This makes correcting a pose straightforward.
        for index, existing_row in enumerate(self.plan['rows']):
            if existing_row['id'] == row['id']:
                self.plan['rows'][index] = row
                self.get_logger().info(f"Updated row {row['id']}")
                return

        self.plan['rows'].append(row)
        self.get_logger().info(f"Added row {row['id']}")

    def upsert_tray(self, raw_tray):
        """Add or replace one tray definition."""
        tray = self.parse_tray(raw_tray, len(self.plan.get('trays', [])))
        if tray is None:
            return

        for index, existing_tray in enumerate(self.plan.get('trays', [])):
            if existing_tray['id'] == tray['id']:
                self.plan['trays'][index] = tray
                self.get_logger().info(f"Updated tray {tray['id']}")
                return

        self.plan.setdefault('trays', []).append(tray)
        self.get_logger().info(f"Added tray {tray['id']}")

    def delete_row(self, row_id):
        """Delete one row by ID."""
        original_count = len(self.plan['rows'])
        self.plan['rows'] = [
            row for row in self.plan['rows']
            if row['id'] != row_id
        ]
        if len(self.plan['rows']) != original_count:
            self.get_logger().info(f'Deleted row {row_id}')

    def delete_tray(self, tray_id):
        """Delete one tray by ID."""
        original_count = len(self.plan.get('trays', []))
        self.plan['trays'] = [
            tray for tray in self.plan.get('trays', [])
            if tray['id'] != tray_id
        ]
        if len(self.plan['trays']) != original_count:
            self.get_logger().info(f'Deleted tray {tray_id}')

    def parse_plan(self, raw_plan):
        """Validate a whole plan object."""
        if isinstance(raw_plan, list):
            raw_plan = {'rows': raw_plan}
        if not isinstance(raw_plan, dict):
            return None

        rows = []
        for index, raw_row in enumerate(raw_plan.get('rows', [])):
            row = self.parse_row(raw_row, index)
            if row is not None:
                rows.append(row)

        trays = []
        for index, raw_tray in enumerate(raw_plan.get('trays', [])):
            tray = self.parse_tray(raw_tray, index)
            if tray is not None:
                trays.append(tray)

        return_home_pose = self.parse_pose(raw_plan.get('return_home_pose'))
        return {
            'trays': trays,
            'rows': rows,
            'return_home_pose': return_home_pose,
        }

    def parse_tray(self, raw_tray, index):
        """Validate one tray definition with A/B/C/D waypoints."""
        if not isinstance(raw_tray, dict):
            self.get_logger().warn(f'Ignoring tray {index}; not an object')
            return None

        tray_id = raw_tray.get('id') or raw_tray.get('tray_id')
        if tray_id is None:
            tray_id = f'tray_{index + 1}'

        waypoints = self.parse_tray_waypoints(raw_tray)
        if waypoints is None:
            self.get_logger().warn(
                f'Ignoring tray {tray_id}; invalid waypoints'
            )
            return None

        return {
            'id': str(tray_id),
            'waypoints': waypoints,
        }

    def parse_tray_waypoints(self, raw_tray):
        """Return a waypoint dictionary from waypoints or nested rows."""
        raw_waypoints = (
            raw_tray.get('waypoints')
            or raw_tray.get('poses')
            or raw_tray.get('points')
        )

        waypoints = {}
        if isinstance(raw_waypoints, dict):
            for name, raw_pose in raw_waypoints.items():
                pose = self.parse_pose(raw_pose)
                if pose is None:
                    return None
                waypoints[str(name).upper()] = pose
        elif isinstance(raw_waypoints, list):
            if len(raw_waypoints) > 4:
                raw_waypoints = raw_waypoints[:4]
            for name, raw_pose in zip(['A', 'B', 'C', 'D'], raw_waypoints):
                pose = self.parse_pose(raw_pose)
                if pose is None:
                    return None
                waypoints[name] = pose
        elif isinstance(raw_tray.get('rows'), list):
            rows = [
                self.parse_row(row, index)
                for index, row in enumerate(raw_tray['rows'])
            ]
            rows = [row for row in rows if row is not None]
            if rows:
                waypoints['A'] = rows[0]['approach_pose']
                waypoints['B'] = rows[0]['scan_end_pose']
            if len(rows) > 1:
                waypoints['C'] = rows[1]['approach_pose']
                waypoints['D'] = rows[1]['scan_end_pose']
        else:
            return None

        if not waypoints:
            return None
        return waypoints

    def parse_row(self, raw_row, index):
        """Validate one row definition."""
        if not isinstance(raw_row, dict):
            self.get_logger().warn(f'Ignoring row {index}; not an object')
            return None

        row_id = raw_row.get('id') or f'row_{index + 1}'
        approach_pose = self.parse_pose(raw_row.get('approach_pose'))

        # goal_pose is a readable alias in the JSON. scan_end_pose is the
        # executor-facing name used by mainloop_node.
        scan_end_pose = self.parse_pose(
            raw_row.get('scan_end_pose') or raw_row.get('goal_pose')
        )
        if approach_pose is None or scan_end_pose is None:
            self.get_logger().warn(
                f'Ignoring row {row_id}; invalid approach or goal pose'
            )
            return None

        return {
            'id': str(row_id),
            'approach_pose': approach_pose,
            'goal_pose': scan_end_pose,
            'scan_end_pose': scan_end_pose,
        }

    @staticmethod
    def parse_pose(raw_pose):
        """Return [x, y, yaw] from a pose array or PoseStamped-like object."""
        if raw_pose is None:
            return None

        if isinstance(raw_pose, (list, tuple)) and len(raw_pose) >= 2:
            try:
                x = float(raw_pose[0])
                y = float(raw_pose[1])
                yaw = None if len(raw_pose) < 3 else float(raw_pose[2])
            except (TypeError, ValueError):
                return None
            return [x, y, yaw]

        return None

    def save_plan(self):
        """Write the generated plan to disk."""
        plan_dir = os.path.dirname(self.plan_path)
        if plan_dir:
            os.makedirs(plan_dir, exist_ok=True)

        with open(self.plan_path, 'w') as file:
            json.dump(self.plan, file, indent=2)

    def timer_callback(self):
        """Periodically republish plan, status, and markers."""
        self.publish_plan()
        self.publish_status()
        self.publish_markers()

    def publish_plan(self, force=False):
        """Publish the full plan for the high-level planner."""
        now = time.time()
        if not force and now - self.last_publish_time < self.publish_period:
            return

        msg = String()
        msg.data = json.dumps(self.plan)
        self.plan_pub.publish(msg)
        self.last_publish_time = now

    def publish_status(self):
        """Publish builder status."""
        trays = self.plan.get('trays', [])
        rows = self.plan.get('rows', [])
        msg = String()
        msg.data = json.dumps({
            'plan_path': self.plan_path,
            'capture_mode': self.capture_mode,
            'tray_count': len(trays),
            'tray_ids': [tray['id'] for tray in trays],
            'incomplete_tray_ids': [
                tray['id'] for tray in trays
                if self.next_missing_tray_segment(tray) is not None
            ],
            'row_count': len(rows),
            'row_ids': [row['id'] for row in rows],
            'pending_tray_id': self.pending_tray_id,
            'pending_row_id': self.pending_row_id,
            'has_return_home_pose': self.plan.get('return_home_pose') is not None,
        })
        self.status_pub.publish(msg)

    def publish_markers(self):
        """Publish RViz markers for the generated plan."""
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        for index, row in enumerate(self.plan_scan_segments()):
            base_id = index * 20

            # Blue = approach pose. Green = goal / scan-end pose.
            # The line shows the intended strafe segment.
            marker_array.markers.append(
                self.make_pose_marker(
                    'generated_approach_pose',
                    base_id,
                    row['approach_pose'],
                    Marker.SPHERE,
                    (0.1, 0.7, 1.0, 0.9),
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_pose_marker(
                    'generated_goal_pose',
                    base_id + 1,
                    row['scan_end_pose'],
                    Marker.CUBE,
                    (0.0, 1.0, 0.25, 0.9),
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_arrow_marker(
                    'generated_approach_arrow',
                    base_id + 2,
                    row['approach_pose'],
                    (0.1, 0.7, 1.0, 0.9),
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_arrow_marker(
                    'generated_goal_arrow',
                    base_id + 3,
                    row['scan_end_pose'],
                    (0.0, 1.0, 0.25, 0.9),
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_line_marker(
                    'generated_scan_line',
                    base_id + 4,
                    row['approach_pose'],
                    row['scan_end_pose'],
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_label_marker(
                    'generated_approach_label',
                    base_id + 5,
                    row['approach_pose'],
                    f"{row['id']} START",
                    (0.1, 0.7, 1.0, 1.0),
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_label_marker(
                    'generated_goal_label',
                    base_id + 6,
                    row['scan_end_pose'],
                    f"{row['id']} END",
                    (0.0, 1.0, 0.25, 1.0),
                    stamp,
                )
            )

        if self.pending_approach_pose is not None:
            # Yellow marker means the user has placed an approach pose, but
            # still needs to place the matching scan-end pose.
            marker_array.markers.append(
                self.make_arrow_marker(
                    'pending_approach_arrow',
                    100000,
                    self.pending_approach_pose,
                    (1.0, 0.9, 0.0, 1.0),
                    stamp,
                )
            )
            marker_array.markers.append(
                self.make_label_marker(
                    'pending_approach_label',
                    100001,
                    self.pending_approach_pose,
                    f'{self.next_pending_capture_label()} PENDING START',
                    (1.0, 0.9, 0.0, 1.0),
                    stamp,
                )
            )

        self.marker_pub.publish(marker_array)

    def plan_scan_segments(self):
        """Return all complete row/segment definitions for visualization."""
        segments = []
        for row in self.plan.get('rows', []):
            segments.append(row)

        for tray in self.plan.get('trays', []):
            tray_id = tray['id']
            waypoints = tray.get('waypoints', {})
            for start_key, end_key in [('A', 'B'), ('C', 'D')]:
                if start_key not in waypoints or end_key not in waypoints:
                    continue
                segment_id = f'{start_key}_to_{end_key}'
                segments.append({
                    'id': f'{tray_id}_{segment_id}',
                    'tray_id': tray_id,
                    'segment_id': segment_id,
                    'approach_pose': waypoints[start_key],
                    'scan_end_pose': waypoints[end_key],
                })

        return segments

    def next_pending_capture_label(self):
        """Return a readable label for the pending pose marker."""
        if self.capture_mode == 'row':
            return self.next_capture_row_id()

        tray_id = self.next_capture_tray_id()
        tray = self.find_tray(tray_id)
        segment = (
            self.next_missing_tray_segment(tray)
            if tray is not None
            else ('A', 'B')
        )
        if segment is None:
            return tray_id
        return f'{tray_id} {segment[0]}->{segment[1]}'

    @staticmethod
    def make_pose_marker(namespace, marker_id, pose, marker_type, color, stamp):
        """Create a marker at one generated pose."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = float(pose[0])
        marker.pose.position.y = float(pose[1])
        marker.pose.position.z = 0.06
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        RowPlanBuilderNode.apply_marker_color(marker, color)
        return marker

    @staticmethod
    def make_arrow_marker(namespace, marker_id, pose, color, stamp):
        """Create a heading arrow at one generated pose."""
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
        marker.scale.x = 0.45
        marker.scale.y = 0.06
        marker.scale.z = 0.06
        RowPlanBuilderNode.apply_marker_color(marker, color)
        return marker

    @staticmethod
    def make_line_marker(namespace, marker_id, start_pose, end_pose, stamp):
        """Create a line marker between generated row poses."""
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.035
        marker.points = [
            Point(x=float(start_pose[0]), y=float(start_pose[1]), z=0.06),
            Point(x=float(end_pose[0]), y=float(end_pose[1]), z=0.06),
        ]
        marker.color.r = 0.1
        marker.color.g = 0.7
        marker.color.b = 1.0
        marker.color.a = 0.8
        return marker

    @staticmethod
    def make_label_marker(namespace, marker_id, pose, text, color, stamp):
        """Create a text label for a generated pose."""
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
        marker.text = text
        RowPlanBuilderNode.apply_marker_color(marker, color)
        return marker

    @staticmethod
    def apply_marker_color(marker, color):
        """Apply an RGBA color tuple to a marker."""
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

    @staticmethod
    def yaw_from_pose_stamped(msg):
        """Extract yaw from a PoseStamped quaternion."""
        # RViz's 2D Goal/Pose tools encode the dragged arrow direction in
        # the quaternion. The mission JSON stores that heading as planar yaw.
        q = msg.pose.orientation
        return math.atan2(
            2 * (q.w * q.z + q.x * q.y),
            1 - 2 * (q.y * q.y + q.z * q.z),
        )

    @staticmethod
    def pose_stamped_to_list(msg):
        """Convert a PoseStamped to [x, y, yaw]."""
        return [
            msg.pose.position.x,
            msg.pose.position.y,
            RowPlanBuilderNode.yaw_from_pose_stamped(msg),
        ]


def main(args=None):
    """Run the row plan builder node."""
    rclpy.init(args=args)
    node = RowPlanBuilderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

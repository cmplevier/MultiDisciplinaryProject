"""Execute navigation and row-scan tasks selected by the planner node."""

import json
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


NAV2_SUCCEEDED = 4


class MainLoopNode(Node):
    """Run one planner-provided task at a time."""

    def __init__(self):
        super().__init__('mdp_mainloop_node')
        self.get_logger().info('MDP mission executor started')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('task_topic', '/planner/next_task')
        self.declare_parameter('status_topic', '/mainloop/status')
        self.declare_parameter('result_topic', '/mainloop/task_result')
        self.declare_parameter('strafe_tolerance', 0.15)
        self.declare_parameter('strafe_gain', 0.2)
        self.declare_parameter('yaw_gain', 0.5)
        self.declare_parameter('max_strafe_speed', 0.1)
        self.declare_parameter('max_yaw_rate', 0.5)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        task_topic = self.get_parameter('task_topic').value
        status_topic = self.get_parameter('status_topic').value
        result_topic = self.get_parameter('result_topic').value

        self.strafe_tolerance = self.get_parameter(
            'strafe_tolerance').value
        self.strafe_gain = self.get_parameter('strafe_gain').value
        self.yaw_gain = self.get_parameter('yaw_gain').value
        self.max_strafe_speed = self.get_parameter(
            'max_strafe_speed').value
        self.max_yaw_rate = self.get_parameter('max_yaw_rate').value

        self.current_pose = None
        self.current_task = None
        self.task_start_time = None
        self.autonomous_enabled = False
        self.state = 'WAITING_FOR_TASK'
        self.nav_goal_handle = None
        self.nav_phase = None
        self.canceling_for_pause = False
        self.strafe_start_yaw = 0.0
        self.last_status_time = 0.0

        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
        )

        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.result_pub = self.create_publisher(String, result_topic, 10)

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

    def pose_callback(self, msg):
        """Store the latest AMCL pose."""
        self.current_pose = msg.pose.pose

    def enable_callback(self, msg):
        """Pause or resume the executor."""
        was_enabled = self.autonomous_enabled
        self.autonomous_enabled = msg.data

        if self.autonomous_enabled and not was_enabled:
            self.get_logger().info('>>> Autonomous Mode: ENABLED')
            if self.current_task is not None:
                self.state = 'TASK_READY'
        elif not self.autonomous_enabled and was_enabled:
            self.get_logger().info('<<< Autonomous Mode: DISABLED (Paused)')
            self.stop_robot()
            if self.nav_goal_handle is not None:
                self.canceling_for_pause = True
                self.nav_goal_handle.cancel_goal_async()
            if self.current_task is not None:
                self.state = 'PAUSED'

        self.publish_status(force=True)

    def task_callback(self, msg):
        """Accept a planner task if the executor is idle."""
        task = self.parse_task(msg.data)
        if task is None:
            return

        if self.current_task is not None:
            current_id = self.current_task['task_id']
            if task['task_id'] != current_id:
                self.get_logger().warn(
                    f"Ignoring task {task['task_id']}; "
                    f"executor is busy with {current_id}"
                )
            return

        self.current_task = task
        self.task_start_time = None
        self.nav_phase = None
        self.state = 'TASK_READY'
        self.get_logger().info(
            f"Accepted task {task['task_id']} ({task['type']})"
        )
        self.publish_status(force=True)

    def parse_task(self, raw_data):
        """Parse and normalize a JSON task message."""
        try:
            task = json.loads(raw_data)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f'Invalid task JSON: {exc}')
            return None

        if not isinstance(task, dict):
            self.get_logger().error('Task message must be a JSON object')
            return None

        task_type = str(task.get('type', 'SCAN_ROW')).upper()
        task_id = task.get('task_id') or task.get('id')
        if not task_id:
            self.get_logger().error('Task is missing task_id')
            return None

        normalized = {
            'task_id': str(task_id),
            'type': task_type,
            'row_id': task.get('row_id'),
            'frame_id': task.get('frame_id', 'map'),
        }

        if task_type == 'SCAN_ROW':
            approach_pose = self.parse_pose(task.get('approach_pose'))
            scan_end_pose = self.parse_pose(task.get('scan_end_pose'))
            if approach_pose is None or scan_end_pose is None:
                self.get_logger().error(
                    f"SCAN_ROW task {task_id} needs approach_pose "
                    'and scan_end_pose'
                )
                return None
            normalized['approach_pose'] = approach_pose
            normalized['scan_end_pose'] = scan_end_pose
        elif task_type == 'NAV_ONLY':
            goal_pose = (
                task.get('goal_pose')
                or task.get('target_pose')
                or task.get('approach_pose')
            )
            goal_pose = self.parse_pose(goal_pose)
            if goal_pose is None:
                self.get_logger().error(
                    f'NAV_ONLY task {task_id} needs goal_pose'
                )
                return None
            normalized['goal_pose'] = goal_pose
        else:
            self.get_logger().error(f'Unsupported task type: {task_type}')
            return None

        return normalized

    @staticmethod
    def parse_pose(raw_pose):
        """Return [x, y, yaw] from a task pose field."""
        if not isinstance(raw_pose, (list, tuple)) or len(raw_pose) < 2:
            return None

        try:
            x = float(raw_pose[0])
            y = float(raw_pose[1])
            yaw = None if len(raw_pose) < 3 else float(raw_pose[2])
        except (TypeError, ValueError):
            return None

        return [x, y, yaw]

    def control_loop(self):
        """Advance the active task state machine."""
        self.publish_status()

        if not self.autonomous_enabled:
            return

        if self.current_task is None:
            self.state = 'WAITING_FOR_TASK'
            return

        if self.current_pose is None:
            self.get_logger().warn(
                'Waiting for robot pose on /amcl_pose...',
                throttle_duration_sec=2.0,
            )
            return

        if self.state == 'TASK_READY':
            self.start_current_task()
        elif self.state == 'STRAFING_ROW':
            self.perform_strafe()

    def start_current_task(self):
        """Start or restart the current task."""
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
            self.get_logger().error('Nav2 action server not available')
            self.finish_current_task(False, 'nav2_action_server_unavailable')
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
            self.finish_current_task(False, 'nav_goal_request_failed')
            return

        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2')
            self.finish_current_task(False, 'nav_goal_rejected')
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
            self.finish_current_task(False, 'nav_result_failed')
            return

        self.nav_goal_handle = None
        phase = self.nav_phase
        self.nav_phase = None

        if self.canceling_for_pause or not self.autonomous_enabled:
            self.canceling_for_pause = False
            self.state = 'PAUSED'
            self.get_logger().info('Navigation paused')
            self.publish_status(force=True)
            return

        if status != NAV2_SUCCEEDED:
            self.get_logger().warn(f'Nav2 ended with status {status}')
            self.finish_current_task(False, f'nav_status_{status}')
            return

        if phase == 'approach' and self.current_task['type'] == 'SCAN_ROW':
            self.get_logger().info(
                f"Reached scan start for {self.current_task['task_id']}"
            )
            self.strafe_start_yaw = self.get_yaw_from_pose(
                self.current_pose
            )
            self.state = 'STRAFING_ROW'
            self.publish_status(force=True)
            return

        self.finish_current_task(True, 'completed')

    def perform_strafe(self):
        """Drive directly toward the row scan end pose."""
        target = self.current_task['scan_end_pose']
        dx = target[0] - self.current_pose.position.x
        dy = target[1] - self.current_pose.position.y
        dist = math.sqrt(dx ** 2 + dy ** 2)

        if dist < self.strafe_tolerance:
            self.get_logger().info(
                f"Task Complete: {self.current_task['task_id']}"
            )
            self.stop_robot()
            self.finish_current_task(True, 'completed')
            return

        yaw = self.get_yaw_from_pose(self.current_pose)
        ex = dx * math.cos(yaw) + dy * math.sin(yaw)
        ey = -dx * math.sin(yaw) + dy * math.cos(yaw)

        target_yaw = target[2]
        if target_yaw is None:
            target_yaw = self.strafe_start_yaw
        yaw_error = (target_yaw - yaw + math.pi) % (2 * math.pi) - math.pi

        twist = Twist()
        twist.linear.x = self.clamp(
            ex * self.strafe_gain,
            -self.max_strafe_speed,
            self.max_strafe_speed,
        )
        twist.linear.y = self.clamp(
            ey * self.strafe_gain,
            -self.max_strafe_speed,
            self.max_strafe_speed,
        )
        twist.angular.z = self.clamp(
            yaw_error * self.yaw_gain,
            -self.max_yaw_rate,
            self.max_yaw_rate,
        )

        self.get_logger().info(
            f"Strafing: dist={dist:.2f}m, "
            f"local_err=({ex:.2f}, {ey:.2f}), "
            f"cmd=({twist.linear.x:.2f}, {twist.linear.y:.2f})",
            throttle_duration_sec=1.0,
        )
        self.cmd_vel_pub.publish(twist)

    def finish_current_task(self, success, reason):
        """Publish a task result and return to idle."""
        if self.current_task is None:
            return

        self.stop_robot()
        duration = 0.0
        if self.task_start_time is not None:
            duration = time.time() - self.task_start_time

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

        if success:
            self.get_logger().info(
                f"Task Result: {task['task_id']} succeeded"
            )
        else:
            self.get_logger().warn(
                f"Task Result: {task['task_id']} failed ({reason})"
            )

        self.current_task = None
        self.task_start_time = None
        self.nav_phase = None
        self.canceling_for_pause = False
        self.state = 'WAITING_FOR_TASK'
        self.publish_status(force=True)

    def publish_status(self, force=False):
        """Publish executor status for the planner."""
        now = time.time()
        if not force and now - self.last_status_time < 0.5:
            return

        self.last_status_time = now
        status = {
            'state': self.state,
            'busy': self.current_task is not None,
            'autonomous_enabled': self.autonomous_enabled,
            'task_id': None,
            'row_id': None,
            'task_type': None,
        }
        if self.current_task is not None:
            status['task_id'] = self.current_task['task_id']
            status['row_id'] = self.current_task.get('row_id')
            status['task_type'] = self.current_task['type']

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def stop_robot(self):
        """Publish a zero velocity command."""
        self.cmd_vel_pub.publish(Twist())

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
    """Run the mainloop executor node."""
    rclpy.init(args=args)
    node = MainLoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

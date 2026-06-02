import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Bool, String
import math
import json
import os
import time
from datetime import datetime

class MainLoopNode(Node):
    def __init__(self):
        super().__init__('mdp_mainloop_node')
        self.get_logger().info('MDP Persistent Mission Node started')

        self.mission_segments = [
            {'type': 'NAV',    'target': [1.439, -1.016, 2.506],  'id': 'APPROACH_ROW_A'},
            {'type': 'STRAFE', 'target': [2.146, 0.189, 2.600],   'id': 'SCAN_ROW_A'},
            {'type': 'NAV',    'target': [1.226, 0.798, -2.324],  'id': 'TRANSITION_TO_B'},
            {'type': 'NAV',    'target': [0.073, -0.709, -0.560], 'id': 'SCAN_ROW_B'},
            {'type': 'NAV',    'target': [1.439, -1.016, 2.506],  'id': 'RETURN_HOME'}
        ]

        self.db_path = os.path.expanduser('~/mdp_ws/mission_history.json')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('clear_history', False)
        self.declare_parameter('strafe_speed', 0.2)
        self.declare_parameter('pose_timeout_sec', 0.5)
        self.declare_parameter('max_retries', 3)
        
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.strafe_speed = self.get_parameter('strafe_speed').value
        self.pose_timeout_sec = self.get_parameter('pose_timeout_sec').value
        self.max_retries = self.get_parameter('max_retries').value

        if self.get_parameter('clear_history').value and os.path.exists(self.db_path):
            os.remove(self.db_path)
            self.get_logger().info('Mission history cleared.')
        
        self.load_history()

        self.current_segment_idx = self.get_first_incomplete_segment()
        self.retry_count = 0
        self.autonomous_enabled = False
        self.segment_start_time = None
        self.nav_goal_handle = None
        
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.stop_pub = self.create_publisher(Twist, 'cmd_vel_stop', 10)
        self.idle_pub = self.create_publisher(Twist, 'cmd_vel_idle', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission_markers', 10)
        self.dashboard_pub = self.create_publisher(String, '/mission_dashboard', 10)
        
        amcl_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, depth=1)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, amcl_qos)
        self.enable_sub = self.create_subscription(Bool, '/autonomous_enabled', self.enable_callback, 10)

        self.current_pose = None
        self.last_pose_time = None
        self.state = 'IDLE'
        self.strafe_start_yaw = 0.0
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def load_history(self):
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    self.history = json.load(f)
            except (FileNotFoundError, json.JSONDecodeError):
                self.history = {}
        else:
            self.history = {}

    def save_history(self, segment_id):
        self.history[segment_id] = {
            'completed_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'duration': f"{time.time() - self.segment_start_time:.1f}s"
        }
        try:
            with open(self.db_path, 'w') as f:
                json.dump(self.history, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to save history: {e}")

    def get_first_incomplete_segment(self):
        for i, seg in enumerate(self.mission_segments):
            if seg['id'] not in self.history:
                return i
        return 0

    def force_stop_robot(self):
        """Publish to the high-priority stop topic to override everything except manual control."""
        msg = Twist()
        for _ in range(5):
            self.stop_pub.publish(msg)
            time.sleep(0.01)

    def enable_callback(self, msg):
        prev = self.autonomous_enabled
        self.autonomous_enabled = msg.data

        if self.autonomous_enabled and not prev:
            self.get_logger().info('>>> Autonomous Mode: ENABLED')
            if self.state in ['IDLE', 'ERROR']:
                self.state = 'START_NEXT_SEGMENT'
                self.retry_count = 0
        elif not self.autonomous_enabled and prev:
            self.get_logger().info('<<< Autonomous Mode: DISABLED (Active Braking)')
            
            if self.nav_goal_handle is not None:
                self.state = 'CANCELLING'
                self.get_logger().info('Sending cancel request to Nav2...')
                self._cancel_future = self.nav_goal_handle.cancel_goal_async()
                self._cancel_future.add_done_callback(self.cancel_done_callback)
            else:
                self.state = 'IDLE'

    def cancel_done_callback(self, future):
        response = future.result()
        if response.return_code == 0:
            self.get_logger().info('Nav2 accepted cancellation. Waiting for robot to halt...')
        else:
            self.get_logger().warn('Nav2 rejected cancellation! Forcing manual stop.')
            self.force_stop_robot()
            self.state = 'IDLE'

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.last_pose_time = self.get_clock().now()

    def control_loop(self):
        self.publish_markers()
        self.publish_dashboard()
        
        # Always publish idle (0) as a baseline.
        self.idle_pub.publish(Twist())

        if not self.autonomous_enabled:
            # Active Braking: Continuously publish 0 to the high-priority stop topic.
            # This ensures the twist_mux ignores Nav2 (80) in favor of this Stop (88).
            self.stop_pub.publish(Twist())
            return

        if self.current_pose is None or self.last_pose_time is None:
            self.get_logger().warn('Waiting for robot pose...', throttle_duration_sec=2.0)
            return

        if self.state == 'START_NEXT_SEGMENT':
            if self.current_segment_idx >= len(self.mission_segments):
                self.get_logger().info('Mission Complete. Restarting loop.')
                self.current_segment_idx, self.history = 0, {}
                if os.path.exists(self.db_path): os.remove(self.db_path)
            
            seg = self.mission_segments[self.current_segment_idx]
            self.segment_start_time = time.time()
            
            if seg['type'] == 'NAV':
                self.get_logger().info(f"Task Start: {seg['id']} (Navigating)")
                self.start_nav_to_target(seg['target'])
                self.state = 'NAVIGATING'
            elif seg['type'] == 'STRAFE':
                self.get_logger().info(f"Task Start: {seg['id']} (Strafing)")
                self.strafe_start_yaw = self.get_yaw_from_pose(self.current_pose)
                self.state = 'STRAFING'

        elif self.state == 'STRAFING':
            self.perform_strafe()

    def start_nav_to_target(self, target):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = target[0], target[1]
        
        yaw = target[2]
        if yaw is not None:
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation = self.current_pose.orientation
        
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 server unavailable!')
            self.handle_task_failure()
            return

        self._send_goal_future = self.nav_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            self.handle_task_failure()
            return
        
        if not self.autonomous_enabled:
            self.nav_goal_handle.cancel_goal_async()
            return

        self._get_result_future = self.nav_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        self.nav_goal_handle = None

        if status == 4: # SUCCEEDED
            self.get_logger().info(f"Task Complete: {self.mission_segments[self.current_segment_idx]['id']}")
            self.save_history(self.mission_segments[self.current_segment_idx]['id'])
            self.current_segment_idx += 1
            self.retry_count = 0
            self.state = 'START_NEXT_SEGMENT'
        elif status == 5: # CANCELED
            self.get_logger().info("Nav2 goal CANCELED. Engaging active brakes.")
            self.force_stop_robot()
            self.state = 'IDLE'
        else:
            self.get_logger().warn(f"Nav2 Task Ended with status {status}")
            self.handle_task_failure()

    def handle_task_failure(self):
        self.retry_count += 1
        if self.retry_count >= self.max_retries:
            self.get_logger().error("Max retries reached. Intervention required.")
            self.state = 'ERROR'
            self.force_stop_robot()
            self.autonomous_enabled = False
        else:
            self.get_logger().info(f"Retrying task ({self.retry_count}/{self.max_retries})...")
            self.state = 'START_NEXT_SEGMENT'

    def perform_strafe(self):
        target = self.mission_segments[self.current_segment_idx]['target']
        dx, dy = target[0] - self.current_pose.position.x, target[1] - self.current_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        
        if dist < 0.15:
            self.get_logger().info(f"Task Complete: {self.mission_segments[self.current_segment_idx]['id']}")
            self.force_stop_robot()
            self.save_history(self.mission_segments[self.current_segment_idx]['id'])
            self.current_segment_idx += 1
            self.retry_count = 0
            self.state = 'START_NEXT_SEGMENT'
            return
            
        yaw = self.get_yaw_from_pose(self.current_pose)
        ex = dx * math.cos(yaw) + dy * math.sin(yaw)
        ey = -dx * math.sin(yaw) + dy * math.cos(yaw)
        
        twist = Twist()
        twist.linear.x = max(min(ex * self.strafe_speed, self.strafe_speed), -self.strafe_speed)
        twist.linear.y = max(min(ey * self.strafe_speed, self.strafe_speed), -self.strafe_speed)
        
        ty = target[2] if target[2] is not None else self.strafe_start_yaw
        ye = (ty - yaw + math.pi) % (2 * math.pi) - math.pi
        twist.angular.z = ye * 0.5
        
        self.cmd_vel_pub.publish(twist)

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def publish_dashboard(self):
        db_lines = ["--- GREENHOUSE SCAN DASHBOARD ---"]
        db_lines.append(f"STATUS: {'[RUNNING]' if self.autonomous_enabled else '[PAUSED/TELEOP]'}")
        db_lines.append(f"STATE: {self.state}")
        db_lines.append("")
        for i, seg in enumerate(self.mission_segments):
            status = "[DONE]" if seg['id'] in self.history else "[    ]"
            if i == self.current_segment_idx and self.autonomous_enabled:
                status = "[ >> ]"
            elif i == self.current_segment_idx:
                status = "[NEXT]"
            line = f"{status} {seg['id']} ({seg['type']})"
            if seg['id'] in self.history:
                line += f" - {self.history[seg['id']]['duration']}"
            db_lines.append(line)
        msg = String(data="\n".join(db_lines))
        self.dashboard_pub.publish(msg)

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, segment in enumerate(self.mission_segments):
            target = segment['target']
            
            # Sphere
            m = Marker()
            m.header.frame_id, m.header.stamp = "map", now
            m.ns, m.id, m.type, m.action = "targets", i, Marker.SPHERE, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = target[0], target[1], 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.2
            m.color.a = 0.8
            
            if segment['id'] in self.history:
                m.color.r = m.color.g = m.color.b = 0.3
            elif i == self.current_segment_idx:
                m.color.g, m.scale.x, m.scale.y, m.scale.z = 1.0, 0.3, 0.3, 0.3
            else:
                m.color.r, m.color.g = 1.0, 0.5
            marker_array.markers.append(m)

            # Label
            lbl = Marker()
            lbl.header.frame_id, lbl.header.stamp = "map", now
            lbl.ns, lbl.id, lbl.type, lbl.action = "labels", i, Marker.TEXT_VIEW_FACING, Marker.ADD
            lbl.pose.position.x, lbl.pose.position.y, lbl.pose.position.z = target[0], target[1], 0.4
            lbl.scale.z, lbl.color.a, lbl.color.r, lbl.color.g, lbl.color.b = 0.15, 1.0, 1.0, 1.0, 1.0
            lbl.text = segment['id']
            marker_array.markers.append(lbl)
            
        self.marker_pub.publish(marker_array)

def main(args=None):
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

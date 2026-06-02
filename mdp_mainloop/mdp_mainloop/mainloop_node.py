import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped, PointStamped
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
        self.get_logger().info('MDP Persistent Mission Node with Debug Logs started')

        # Define mission segments
        self.mission_segments = [
            {'type': 'NAV',    'target': [1.439, -1.016, 2.506],  'id': 'APPROACH_ROW_A'},
            {'type': 'STRAFE', 'target': [2.146, 0.189, 2.600],   'id': 'SCAN_ROW_A'},
            {'type': 'NAV',    'target': [1.226, 0.798, -2.324],  'id': 'TRANSITION_TO_B'},
            {'type': 'NAV', 'target': [0.073, -0.709, -0.560], 'id': 'SCAN_ROW_B'},
            {'type': 'NAV',    'target': [1.439, -1.016, 2.506],  'id': 'RETURN_HOME'}
        ]

        # Persistence setup
        self.db_path = os.path.expanduser('~/mdp_ws/mission_history.json')

        # Parameters
        self.declare_parameter('cmd_vel_topic', '/cmd_vel_nav')
        self.declare_parameter('clear_history', False)
        
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        clear_history = self.get_parameter('clear_history').value

        if clear_history:
            if os.path.exists(self.db_path):
                os.remove(self.db_path)
                self.get_logger().info('Mission history cleared by parameter.')
        
        self.load_history()

        self.current_segment_idx = self.get_first_incomplete_segment()
        self.autonomous_enabled = False
        self.segment_start_time = None
        self.nav_goal_handle = None
        self._send_goal_future = None
        
        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/mission_markers', 10)
        self.dashboard_pub = self.create_publisher(String, '/mission_dashboard', 10)
        
        # Subscribers
        amcl_qos = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, depth=1)
        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_callback, amcl_qos)
        self.enable_sub = self.create_subscription(Bool, '/autonomous_enabled', self.enable_callback, 10)
        self.click_sub = self.create_subscription(PointStamped, '/clicked_point', self.click_callback, 10)

        self.current_pose = None
        self.state = 'IDLE'
        self.strafe_start_yaw = 0.0
        
        self.timer = self.create_timer(0.05, self.control_loop)

    def load_history(self):
        if os.path.exists(self.db_path):
            try:
                with open(self.db_path, 'r') as f:
                    self.history = json.load(f)
            except:
                self.history = {}
        else:
            self.history = {}

    def save_history(self, segment_id):
        self.history[segment_id] = {
            'completed_at': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
            'duration': f"{time.time() - self.segment_start_time:.1f}s"
        }
        with open(self.db_path, 'w') as f:
            json.dump(self.history, f, indent=4)

    def get_first_incomplete_segment(self):
        for i, seg in enumerate(self.mission_segments):
            if seg['id'] not in self.history:
                return i
        return 0 # Default to start if all done

    def enable_callback(self, msg):
        previous_enabled = self.autonomous_enabled
        self.autonomous_enabled = msg.data

        if self.autonomous_enabled and not previous_enabled:
            self.get_logger().info('>>> Autonomous Mode: ENABLED')
            if self.state == 'IDLE':
                self.state = 'START_NEXT_SEGMENT'
        elif not self.autonomous_enabled and previous_enabled:
            self.get_logger().info('<<< Autonomous Mode: DISABLED (Paused)')
            # Immediately send stop command
            stop_msg = Twist()
            self.cmd_vel_pub.publish(stop_msg)
            
            # Cancel active Nav2 goal if it exists
            if self.nav_goal_handle is not None:
                self.get_logger().info('Requesting Nav2 goal cancellation...')
                self.nav_goal_handle.cancel_goal_async()
            # If a goal was recently sent but handle is not yet available, 
            # the goal_response_callback will handle it.

    def click_callback(self, msg):
        click_x, click_y = msg.point.x, msg.point.y
        best_dist, best_idx = float('inf'), self.current_segment_idx
        
        for i, seg in enumerate(self.mission_segments):
            t = seg['target']
            dist = math.sqrt((click_x - t[0])**2 + (click_y - t[1])**2)
            if dist < best_dist:
                best_dist, best_idx = dist, i
        
        if best_dist < 2.0:
            seg_id = self.mission_segments[best_idx]['id']
            self.get_logger().info(f"Manual Task Selection: {seg_id}")
            
            # Cancel current work
            if self.nav_goal_handle is not None:
                self.nav_goal_handle.cancel_goal_async()
            
            self.current_segment_idx = best_idx
            if seg_id in self.history:
                del self.history[seg_id]
            
            if self.autonomous_enabled:
                self.state = 'START_NEXT_SEGMENT'
            self.publish_markers()
            self.publish_dashboard()

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

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
        msg = String()
        msg.data = "\n".join(db_lines)
        self.dashboard_pub.publish(msg)

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, segment in enumerate(self.mission_segments):
            target = segment['target']
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = "map", self.get_clock().now().to_msg()
            marker.ns, marker.id, marker.type, marker.action = "mission_targets", i, Marker.SPHERE, Marker.ADD
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(target[0]), float(target[1]), 0.05
            marker.pose.orientation.w, marker.scale.x, marker.scale.y, marker.scale.z = 1.0, 0.2, 0.2, 0.2
            marker.color.a = 0.8
            if segment['id'] in self.history:
                marker.color.r, marker.color.g, marker.color.b = 0.3, 0.3, 0.3
            elif i == self.current_segment_idx:
                marker.color.r, marker.color.g, marker.color.b, marker.scale.x, marker.scale.y, marker.scale.z = 0.0, 1.0, 0.0, 0.3, 0.3, 0.3
            else:
                marker.color.r, marker.color.g, marker.color.b = 1.0, 0.5, 0.0
            marker_array.markers.append(marker)

            yaw = target[2]
            if yaw is not None:
                arrow = Marker()
                arrow.header.frame_id, arrow.header.stamp = "map", self.get_clock().now().to_msg()
                arrow.ns, arrow.id, arrow.type, arrow.action = "wanted_poses", i, Marker.ARROW, Marker.ADD
                arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = float(target[0]), float(target[1]), 0.1
                arrow.pose.orientation.z, arrow.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
                arrow.scale.x, arrow.scale.y, arrow.scale.z, arrow.color.a = 0.4, 0.05, 0.05, 1.0
                arrow.color.r, arrow.color.g, arrow.color.b = marker.color.r, marker.color.g, marker.color.b
                marker_array.markers.append(arrow)

            label = Marker()
            label.header.frame_id, label.header.stamp = "map", self.get_clock().now().to_msg()
            label.ns, label.id, label.type, label.action = "mission_labels", i, Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y, label.pose.position.z = float(target[0]), float(target[1]), 0.4
            label.scale.z, label.color.a, label.color.r, label.color.g, label.color.b, label.text = 0.15, 1.0, 1.0, 1.0, 1.0, segment['id']
            marker_array.markers.append(label)
        self.marker_pub.publish(marker_array)

    def control_loop(self):
        self.publish_markers()
        self.publish_dashboard()
        
        if not self.autonomous_enabled:
            # If we were previously in a moving state, ensure we stay stopped until cancellation is complete
            if self.nav_goal_handle is not None or self.state == 'STRAFING':
                self.cmd_vel_pub.publish(Twist())
                self.get_logger().info('Stopping robot and cancelling active tasks...', throttle_duration_sec=2.0)
            return

        if self.current_pose is None:
            self.get_logger().warn('Waiting for robot pose on /amcl_pose...', throttle_duration_sec=2.0)
            return

        if self.state == 'START_NEXT_SEGMENT':

            if self.current_segment_idx >= len(self.mission_segments):
                self.get_logger().info('Loop Finished. Restarting...')
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
        goal.pose.header.frame_id, goal.pose.header.stamp = 'map', self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(target[0]), float(target[1])
        yaw = target[2]
        if yaw is not None:
            goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation = self.current_pose.orientation
        
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('Nav2 action server not available! Skipping NAV task.')
            self.state = 'START_NEXT_SEGMENT'
            return

        self._send_goal_future = self.nav_client.send_goal_async(goal)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.nav_goal_handle = future.result()
        if not self.nav_goal_handle.accepted:
            self.get_logger().error('Goal REJECTED by Nav2')
            self.state = 'START_NEXT_SEGMENT'
            return
        
        # If autonomous mode was disabled while waiting for acceptance
        if not self.autonomous_enabled:
            self.get_logger().warn('Goal accepted but autonomous mode is DISABLED. Cancelling immediately.')
            self.nav_goal_handle.cancel_goal_async()
            return

        self._get_result_future = self.nav_goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        if status == 4: # SUCCEEDED
            self.get_logger().info(f"Task Complete: {self.mission_segments[self.current_segment_idx]['id']}")
            self.save_history(self.mission_segments[self.current_segment_idx]['id'])
            self.current_segment_idx += 1
        else:
            self.get_logger().warn(f"Task Ended with status {status}")
            # If aborted/cancelled, we stay on this index to allow retry
        
        self.nav_goal_handle = None
        self.state = 'START_NEXT_SEGMENT'

    def perform_strafe(self):
        target = self.mission_segments[self.current_segment_idx]['target']
        dx, dy = target[0] - self.current_pose.position.x, target[1] - self.current_pose.position.y
        dist = math.sqrt(dx**2 + dy**2)
        if dist < 0.15:
            self.get_logger().info(f"Task Complete: {self.mission_segments[self.current_segment_idx]['id']}")
            self.cmd_vel_pub.publish(Twist())
            self.save_history(self.mission_segments[self.current_segment_idx]['id'])
            self.current_segment_idx += 1
            self.state = 'START_NEXT_SEGMENT'
            return
        yaw = self.get_yaw_from_pose(self.current_pose)
        ex, ey = dx * math.cos(yaw) + dy * math.sin(yaw), -dx * math.sin(yaw) + dy * math.cos(yaw)
        twist = Twist()
        twist.linear.x = max(min(ex * 0.2, 0.1), -0.1)
        twist.linear.y = max(min(ey * 0.2, 0.1), -0.1)
        ty = target[2] if target[2] is not None else self.strafe_start_yaw
        ye = (ty - yaw + math.pi) % (2 * math.pi) - math.pi
        twist.angular.z = ye * 0.5
        self.get_logger().info(f"Strafing: dist={dist:.2f}m, local_err=({ex:.2f}, {ey:.2f}), cmd=({twist.linear.x:.2f}, {twist.linear.y:.2f})", throttle_duration_sec=1.0)
        self.cmd_vel_pub.publish(twist)

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

def main(args=None):
    rclpy.init(args=args)
    node = MainLoopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Send a final stop command on exit
        node.get_logger().info('MainLoopNode stopping... sending final stop command.')
        stop_msg = Twist()
        node.cmd_vel_pub.publish(stop_msg)
        # Allow a small moment for the message to be sent
        time.sleep(0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

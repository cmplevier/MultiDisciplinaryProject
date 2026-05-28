import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from visualization_msgs.msg import Marker, MarkerArray
import math
import time

class MainLoopNode(Node):
    def __init__(self):
        super().__init__('mdp_mainloop_node')
        self.get_logger().info('MDP Main Loop Node with Strafing has started')

        # Parameters for the rectangle (example values, can be overridden)
        self.declare_parameter('c1', [1.0, 1.0, 0.0])
        self.declare_parameter('c2', [1.0, -1.0, 0.0])
        self.declare_parameter('c3', [-1.0, -1.0, 3.14159/2])  # 90 degrees in radians
        self.declare_parameter('c4', [-1.0, 0.75, 3.14159])  # 180 degrees in radians
        
        self.corners = []
        for i in range(1, 5):
            val = self.get_parameter(f'c{i}').value
            if len(val) == 2:
                self.corners.append([float(val[0]), float(val[1]), None])
            else:
                self.corners.append([float(val[0]), float(val[1]), float(val[2])])

        # Action client for Nav2
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # QoS profile for AMCL (Transient Local is required to match AMCL)
        amcl_qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            depth=1
        )

        # Publisher for custom strafing commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)

        # Publisher for waypoint markers
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)
        
        # Subscriber for robot pose with matching QoS
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.pose_callback,
            amcl_qos
        )

        self.current_pose = None
        self.state = 'WAITING_FOR_POSE'
        self.current_corner_idx = 0
        self.strafe_start_yaw = 0.0
        
        # Timer for the state machine
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().info('First pose received!')
        self.current_pose = msg.pose.pose

    def publish_markers(self):
        marker_array = MarkerArray()
        for i, corner in enumerate(self.corners):
            # Sphere marker for position
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = corner[0]
            marker.pose.position.y = corner[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            if i == self.current_corner_idx and self.state != 'FINISHED':
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            marker_array.markers.append(marker)

            # Arrow marker for orientation (only if specified)
            yaw = corner[2]
            if yaw is not None:
                arrow_marker = Marker()
                arrow_marker.header.frame_id = "map"
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.ns = "waypoint_orientations"
                arrow_marker.id = i
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.pose.position.x = corner[0]
                arrow_marker.pose.position.y = corner[1]
                arrow_marker.pose.position.z = 0.05
                
                arrow_marker.pose.orientation.z = math.sin(yaw / 2.0)
                arrow_marker.pose.orientation.w = math.cos(yaw / 2.0)
                
                arrow_marker.scale.x = 0.3 # Length
                arrow_marker.scale.y = 0.05 # Width
                arrow_marker.scale.z = 0.05 # Height
                arrow_marker.color.a = 1.0
                if i == self.current_corner_idx and self.state != 'FINISHED':
                    arrow_marker.color.r = 0.0
                    arrow_marker.color.g = 1.0
                    arrow_marker.color.b = 0.0
                else:
                    arrow_marker.color.r = 1.0
                    arrow_marker.color.g = 0.0
                    arrow_marker.color.b = 0.0
                marker_array.markers.append(arrow_marker)
            
            # Add text label
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = corner[0]
            text_marker.pose.position.y = corner[1]
            text_marker.pose.position.z = 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.15
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.text = f"C{i+1}"
            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def control_loop(self):
        self.publish_markers()
        if self.current_pose is None:
            return

        if self.state == 'WAITING_FOR_POSE':
            self.get_logger().info('Pose received, starting rectangle mission')
            self.state = 'GO_TO_CORNER'
            self.current_corner_idx = 0
            self.start_nav_to_corner()

        elif self.state == 'STRAFING':
            self.perform_strafe()

    def start_nav_to_corner(self):
        target = self.corners[self.current_corner_idx]
        self.get_logger().info(f'Navigating to Corner {self.current_corner_idx + 1}: {target}')
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = target[0]
        goal_msg.pose.pose.position.y = target[1]
        
        yaw = target[2]
        if yaw is not None:
            goal_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            # If no yaw specified, use current orientation for Nav2 goal to avoid unnecessary rotation
            goal_msg.pose.pose.orientation = self.current_pose.orientation

        self.nav_client.wait_for_server()
        self._send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(f'Reached Corner {self.current_corner_idx + 1}')
        
        # Logic for state transition
        if self.current_corner_idx == 0:
            # We reached C1, now strafe to C2
            self.state = 'STRAFING'
            self.current_corner_idx = 1
            self.strafe_start_yaw = self.get_yaw_from_pose(self.current_pose)
            self.get_logger().info(f'Starting strafe to Corner 2. Locking yaw at {self.strafe_start_yaw:.2f}')
        else:
            # Standard navigation to next corner
            self.current_corner_idx = (self.current_corner_idx + 1) % 4
            if self.current_corner_idx == 0:
                self.get_logger().info('Mission complete!')
                self.state = 'FINISHED'
            else:
                self.start_nav_to_corner()

    def perform_strafe(self):
        target = self.corners[self.current_corner_idx] # Corner 2
        
        dx = target[0] - self.current_pose.position.x
        dy = target[1] - self.current_pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 0.1: # Goal tolerance
            self.get_logger().info('Strafe complete, reached Corner 2')
            self.cmd_vel_pub.publish(Twist()) # Stop
            self.state = 'GO_TO_CORNER'
            self.current_corner_idx = 2
            self.start_nav_to_corner()
            return

        # Simple P-control for Y (strafing)
        # Note: This assumes the robot is oriented correctly at C1 
        # (facing along X axis, so Y error is lateral)
        # For a truly robust controller, we would transform the error into the robot frame.
        
        # Transform global error to robot frame
        yaw = self.get_yaw_from_pose(self.current_pose)
        
        # Error in robot frame
        err_robot_x = dx * math.cos(yaw) + dy * math.sin(yaw)
        err_robot_y = -dx * math.sin(yaw) + dy * math.cos(yaw)

        twist = Twist()
        kp_lin = 0.5
        kp_ang = 1.0
        
        # We want to STRAFE, so we use linear.y for the Y error
        # and linear.x for the X error (to stay on the line)
        # and angular.z to lock the heading
        
        twist.linear.x = err_robot_x * kp_lin
        twist.linear.y = err_robot_y * kp_lin
        
        # Lock heading to target yaw, or current yaw if unspecified
        target_yaw = target[2] if target[2] is not None else self.strafe_start_yaw
        yaw_error = target_yaw - yaw
        # Normalize yaw error
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi
        twist.angular.z = yaw_error * kp_ang

        # Limit speeds
        twist.linear.x = max(min(twist.linear.x, 0.2), -0.2)
        twist.linear.y = max(min(twist.linear.y, 0.2), -0.2)

        self.cmd_vel_pub.publish(twist)

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
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

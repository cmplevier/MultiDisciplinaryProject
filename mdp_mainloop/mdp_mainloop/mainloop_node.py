import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
import math
import time

class MainLoopNode(Node):
    def __init__(self):
        super().__init__('mdp_mainloop_node')
        self.get_logger().info('MDP Main Loop Node with Strafing has started')

        # Parameters for the rectangle (example values, can be overridden)
        self.declare_parameter('c1', [1.0, 1.0])
        self.declare_parameter('c2', [1.0, -1.0])
        self.declare_parameter('c3', [-1.0, -1.0])
        self.declare_parameter('c4', [-1.0, 0.75])
        
        self.corners = [
            self.get_parameter('c1').value,
            self.get_parameter('c2').value,
            self.get_parameter('c3').value,
            self.get_parameter('c4').value
        ]

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
        
        # Timer for the state machine
        self.timer = self.create_timer(0.1, self.control_loop)

    def pose_callback(self, msg):
        if self.current_pose is None:
            self.get_logger().info('First pose received!')
        self.current_pose = msg.pose.pose

    def control_loop(self):
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
        goal_msg.pose.pose.position.x = float(target[0])
        goal_msg.pose.pose.position.y = float(target[1])
        goal_msg.pose.pose.orientation.w = 1.0 # Default orientation

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
            self.get_logger().info('Starting strafe to Corner 2')
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
        
        # Lock heading to 0 (or whatever heading it had at C1)
        # For simplicity, let's assume heading 0 for the rectangle.
        target_yaw = 0.0 
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

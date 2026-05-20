import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
# from lupin_greenhouse_msgs.msg import ... # Import custom messages as needed

class MainLoopNode(Node):
    def __init__(self):
        super().__init__('mdp_mainloop_node')
        self.get_logger().info('MDP Main Loop Node has been started')
        
        # Publisher for navigation goals
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        
        # Example timer to periodically do something
        # self.timer = self.create_timer(1.0, self.timer_callback)

    def send_waypoint(self, x, y, z=0.0, ox=0.0, oy=0.0, oz=0.0, ow=1.0):
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.position.z = z
        goal_msg.pose.orientation.x = ox
        goal_msg.pose.orientation.y = oy
        goal_msg.pose.orientation.z = oz
        goal_msg.pose.orientation.w = ow
        
        self.get_logger().info(f'Sending waypoint: x={x}, y={y}')
        self.goal_pub.publish(goal_msg)

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

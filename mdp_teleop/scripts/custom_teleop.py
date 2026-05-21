#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class CustomTeleopJoy(Node):
    def __init__(self):
        super().__init__('custom_teleop_joy')
        
        # Declare parameters (can also load these from yaml)
        self.declare_parameter('axis_l_x', 0)               # Left stick Left/Right
        self.declare_parameter('axis_l_y', 1)               # Left stick Up/Down
        self.declare_parameter('axis_r_x', 3)               # Right stick Left/Right
        self.declare_parameter('axis_r_y', 4)               # Right stick Up/Down

        self.declare_parameter('btn_enable', 5)             # R1 on PS4
        self.declare_parameter('btn_enable_holonomic', 4)   # L1 on PS4
        
        self.declare_parameter('scale_linear_x', 1.0)
        self.declare_parameter('scale_linear_y', 1.0)
        self.declare_parameter('scale_angular_z', 2.0)

        # Get parameters
        self.axis_l_x = self.get_parameter('axis_l_x').value
        self.axis_l_y = self.get_parameter('axis_l_y').value    # x velocity!  
        self.axis_r_x = self.get_parameter('axis_r_x').value
        self.axis_r_y = self.get_parameter('axis_r_y').value

        self.btn_enable = self.get_parameter('btn_enable').value
        self.btn_holonomic = self.get_parameter('btn_enable_holonomic').value

        self.scale_linear_x = self.get_parameter('scale_linear_x').value
        self.scale_linear_y = self.get_parameter('scale_linear_y').value
        self.scale_angular_z = self.get_parameter('scale_angular_z').value

        # Publishers and Subscribers
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 2) # Any reason to have 10, 1 would be best?
        # self.arm_pub = self.create_publisher()
        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10) # Same here

    def joy_callback(self, msg):
        twist = Twist()

        if msg.buttons[self.btn_enable] == 1:
            # Deadman switch active

            twist.linear.x = msg.axes[self.axis_l_y] * self.scale_linear_x

            if msg.buttons[self.btn_holonomic] == 1:
                # Holonomic mode active
                twist.linear.y = msg.axes[self.axis_l_x] * self.scale_linear_y
                twist.angular.z = 0.0
            
            else:
                # Non holonomic mode
                twist.linear.y = 0.0
                twist.angular.z = msg.axes[self.axis_l_x] * self.scale_angular_z

        else:
            # Deadman switch off
            twist.linear.x = 0.0
            twist.linear.y = 0.0
            twist.angular.z = 0.0                
            
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTeleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
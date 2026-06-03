#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import JointTrajectoryControllerState
from builtin_interfaces.msg import Duration

class CustomTeleopJoy(Node):
    def __init__(self):
        super().__init__('custom_teleop_joy')
        
        # Declare parameters (can also load these from yaml)
        # Base params
        self.declare_parameter('axis_l_x', 0)               # Left stick Left/Right
        self.declare_parameter('axis_l_y', 1)               # Left stick Up/Down
        self.declare_parameter('axis_r_x', 3)               # Right stick Left/Right
        self.declare_parameter('axis_r_y', 4)               # Right stick Up/Down

        self.declare_parameter('btn_enable', 5)             # R1 on PS4
        self.declare_parameter('btn_enable_holonomic', 4)   # L1 on PS4
        
        self.declare_parameter('scale_linear_x', 1.0)
        self.declare_parameter('scale_linear_y', 1.0)
        self.declare_parameter('scale_angular_z', 2.0)

        # Arm params
        # self.declare_parameter('axis_dpad_x', 6)            # D-pad Left/Right
        # self.declare_parameter('axis_dpad_y', 7)            # D-pad Up/Down

        self.declare_parameter('btn_triangle', 2)           # Elbow +
        self.declare_parameter('btn_cross', 0)              # Elbow -
        self.declare_parameter('btn_circle', 1)             # Wrist +
        self.declare_parameter('btn_square', 3)             # Wrist -
        
        self.declare_parameter('arm_step_size', 0.15)
        self.declare_parameter('arm_step_time', 0.20)

        # Joint Physical Limits [Pan, Lift, Elbow, Wrist]
        self.declare_parameter('joint_limit_min', [-1.57, -1.57, -1.57, -1.57])
        self.declare_parameter('joint_limit_max', [ 1.57,  1.57,  1.57,  1.57])


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

        # self.axis_dpad_x = self.get_parameter('axis_dpad_x').value
        # self.axis_dpad_y = self.get_parameter('axis_dpad_y').value
        self.btn_triangle = self.get_parameter('btn_triangle').value
        self.btn_cross = self.get_parameter('btn_cross').value
        self.btn_circle = self.get_parameter('btn_circle').value
        self.btn_square = self.get_parameter('btn_square').value

        self.arm_step = self.get_parameter('arm_step_size').value
        self.arm_time = self.get_parameter('arm_step_time').value
        
        self.joint_limit_min = self.get_parameter('joint_limit_min').value
        self.joint_limit_max = self.get_parameter('joint_limit_max').value



        self.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_joint']
        self.current_arm_positions = [0.0, 0.0, 0.0, 0.0]
        self.arm_initialized = False

        # Publishers and Subscribers
        self.base_pub = self.create_publisher(Twist, '/cmd_vel', 2) # Any reason to have 10, 1 would be best?
        self.arm_pub = self.create_publisher(JointTrajectory, '/mirte_master_arm_controller/joint_trajectory', 2)

        # self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10) # Same here
        self.joy_sub = self.create_subscription(Joy, '/joy', self.joy_callback, 2)
        self.arm_state_sub = self.create_subscription(
            JointTrajectoryControllerState, 
            '/mirte_master_arm_controller/state', 
            self.arm_state_callback, 
            2
        )

    def arm_state_callback(self, msg):
        """Continuously records where the hardware actually is."""
        if len(msg.actual.positions) >= len(self.joint_names):
            self.current_arm_positions = list(msg.actual.positions)
            self.arm_initialized = True

    def joy_callback(self, msg):
        
        # BASE CONTROL
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
            
        self.base_pub.publish(twist)

        # ARM CONTROL
        if msg.buttons[self.btn_enable] == 1 and self.arm_initialized:

            new_positions = list(self.current_arm_positions)
            arm_command_triggered = False

            # -- Joint 0: Shoulder Pan (D-pad Left/Right) --
            if msg.axes[self.axis_r_x] > 0.2:   # Left
                new_positions[0] += self.arm_step
                arm_command_triggered = True
            elif msg.axes[self.axis_r_x] < -0.2: # Right
                new_positions[0] -= self.arm_step
                arm_command_triggered = True

            # -- Joint 1: Shoulder Lift (D-pad Up/Down) --
            if msg.axes[self.axis_r_y] > 0.5:   # Up
                new_positions[1] -= self.arm_step
                arm_command_triggered = True
            elif msg.axes[self.axis_r_y] < -0.5: # Down
                new_positions[1] += self.arm_step
                arm_command_triggered = True

            # -- Joint 2: Elbow (Triangle / Cross) --
            if msg.buttons[self.btn_triangle] == 1:
                new_positions[2] -= self.arm_step
                arm_command_triggered = True
            elif msg.buttons[self.btn_cross] == 1:
                new_positions[2] += self.arm_step
                arm_command_triggered = True

            # -- Joint 3: Wrist (Circle / Square) --
            if msg.buttons[self.btn_circle] == 1:
                new_positions[3] += self.arm_step
                arm_command_triggered = True
            elif msg.buttons[self.btn_square] == 1:
                new_positions[3] -= self.arm_step
                arm_command_triggered = True


            if arm_command_triggered:
                # Clamp all positions within defined limits
                for i in range(len(new_positions)):
                    new_positions[i] = max(self.joint_limit_min[i], min(new_positions[i], self.joint_limit_max[i]))

                traj_msg = JointTrajectory()
                traj_msg.joint_names = self.joint_names
                
                point = JointTrajectoryPoint()
                point.positions = new_positions
                
                sec = int(self.arm_time)
                nanosec = int((self.arm_time - sec) * 1e9)
                point.time_from_start = Duration(sec=sec, nanosec=nanosec)
                
                traj_msg.points.append(point)
                self.arm_pub.publish(traj_msg)

def main(args=None):
    rclpy.init(args=args)
    node = CustomTeleopJoy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    cmd_vel_remap = DeclareLaunchArgument(
        'cmd_vel_remap', 
        default_value='/mirte_base_controller/cmd_vel',
        description='Expose topic remap at launch'    
    )

    joy_config_arg = DeclareLaunchArgument(
        'joy_config',
        default_value='custom_ps5.yaml',
        description='Type of joystick config file to load (e.g., ps4, xbox)'
    )

    # Path to config file
    config_file = PathJoinSubstitution([
        FindPackageShare('mdp_teleop'), 'config', LaunchConfiguration('joy_config')
    ])
    
    # Node that reads joystick inputs and publishes them
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    # Node that reads the joy_node messages and translates them into geometry_msgs/msg/Twist
    teleop_node = Node(
        package="mdp_teleop",
        executable="custom_teleop.py", # Direct script file target
        name="teleop_joy_node",
        remappings=[("/cmd_vel", LaunchConfiguration('cmd_vel_remap'))],
        parameters=[config_file, {'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_arg,
        cmd_vel_remap,
        joy_config_arg,
        joy_node,
        teleop_node
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    clear_history = LaunchConfiguration('clear_history')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel_nav',
            description='Topic for velocity commands'),

        DeclareLaunchArgument(
            'clear_history',
            default_value='false',
            description='Clear mission history on startup'),

        Node(
            package='mdp_mainloop',
            executable='mainloop_node',
            name='mdp_mainloop_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'cmd_vel_topic': cmd_vel_topic},
                {'clear_history': clear_history},
            ]
        )
    ])

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mdp_mainloop',
            executable='mainloop_node',
            name='mdp_mainloop_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ]
        )
    ])

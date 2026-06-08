from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='cpu',
                              description='YOLO device (cpu or cuda:0)'),

        Node(
            package='mdp_perception',
            executable='perception_node',
            name='perception_node',
            output='screen',
            parameters=[{'device': LaunchConfiguration('device')}],
        ),

        Node(
            package='state_node',
            executable='state_node',
            name='state_node',
            output='screen',
        ),
    ])

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cpu',
        description='Torch device for YOLO inference (e.g. cpu, 0, cuda:0)',
    )

    perception = Node(
        package='mdp_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{'device': LaunchConfiguration('device')}],
    )

    image_view = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view',
             '/perception/debug_image/compressed'],
        output='screen',
    )

    return LaunchDescription([device_arg, perception, image_view])

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Path to your custom RViz config
    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('mdp_slam'),
        'rviz',
        'mapping.rviz'
    ])

    # Path to SLAM toolbox config
    slam_config_path = PathJoinSubstitution([
        FindPackageShare('mdp_slam'),
        'config',
        'mapping_params_online_async.yaml'
    ])

    # Include the SLAM Toolbox launch
    slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={'slam_params_file': slam_config_path, 'use_sim_time': 'true'}.items()
    )

    # Command to launch RViz with your config
    rviz_cmd = ExecuteProcess(
        cmd=['rviz2', '-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        slam_toolbox_cmd,
        rviz_cmd
    ])

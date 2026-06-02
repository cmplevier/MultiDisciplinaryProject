from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare('mdp_bringup'),
                    'config',
                    'twist_mux.yaml'
                ]),
                {'use_sim_time': False}
            ],
            remappings=[
                ('cmd_vel_out', '/mirte_base_controller/cmd_vel')
            ]
        )
    ])

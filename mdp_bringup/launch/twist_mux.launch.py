from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    output_topic = PythonExpression([
        "'/mirte_base_controller/cmd_vel_unstamped' if '", 
        use_sim_time, 
        "'.lower() in ['true', '1', 'yes'] else '/mirte_base_controller/cmd_vel'"
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),
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
                {'use_sim_time': use_sim_time}
            ],
            remappings=[
                ('cmd_vel_out', output_topic)
            ]
        )
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    global_localization = LaunchConfiguration('global_localization')

    nav2_bringup_dir = FindPackageShare('nav2_bringup')

    default_map_path = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'maps',
        #'my_map_save.yaml'
        'asym_map.yaml'
    ])

    default_params_path = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'config',
        'amcl_params.yaml'
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'rviz',
        'localizing.rviz'
    ])

    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                nav2_bringup_dir,
                'launch',
                'localization_launch.py'
            ])
        ),
        launch_arguments={
            'map': map_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    global_localization_cmd = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2',
                    'service',
                    'call',
                    '/reinitialize_global_localization',
                    'std_srvs/srv/Empty',
                    '{}'
                ],
                condition=IfCondition(global_localization),
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=default_map_path,
            description='Full path to map yaml file'
        ),

        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_path,
            description='Full path to AMCL/Nav2 localization params file'
        ),

        DeclareLaunchArgument(
            'global_localization',
            default_value='false',
            description='Spread AMCL particles over the whole map after startup. Use false when the initial pose is known to be accurate.'
        ),

        localization_cmd,
        global_localization_cmd,
        rviz_cmd
    ])

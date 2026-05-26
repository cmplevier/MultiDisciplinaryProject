from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    localization_params_file = LaunchConfiguration('localization_params_file')
    navigation_params_file = LaunchConfiguration('navigation_params_file')
    odom_topic = LaunchConfiguration('odom_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    global_localization = LaunchConfiguration('global_localization')
    localization_bond_timeout = LaunchConfiguration('localization_bond_timeout')
    autostart = LaunchConfiguration('autostart')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

    combined_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_navigation'),
                'launch',
                'nav2_with_localization.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'localization_params_file': localization_params_file,
            'navigation_params_file': navigation_params_file,
            'odom_topic': odom_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'global_localization': global_localization,
            'localization_bond_timeout': localization_bond_timeout,
            'autostart': autostart,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'launch_rviz': launch_rviz,
            'rviz_config_file': rviz_config_file,
        }.items()
    )

    default_map_file = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'maps',
        'asym_map.yaml'
    ])
    default_localization_params = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'config',
        'amcl_params.yaml'
    ])
    default_navigation_params = PathJoinSubstitution([
        FindPackageShare('mdp_navigation'),
        'config',
        'nav2_params.yaml'
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('mdp_navigation'),
        'rviz',
        'combined.rviz'
    ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'odom_topic',
            default_value=PythonExpression([
                "'/odom' if '",
                use_sim_time,
                "'.lower() in ['true', '1', 'yes'] else '/mirte_base_controller/odom'",
            ]),
            description='Odometry topic for Nav2. Defaults to /odom in simulation and /mirte_base_controller/odom on the real robot.'
        ),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value=PythonExpression([
                "'/mirte_base_controller/cmd_vel_unstamped' if '",
                use_sim_time,
                "'.lower() in ['true', '1', 'yes'] else '/mirte_base_controller/cmd_vel'",
            ]),
            description='Topic where Nav2 controller commands are published. Defaults to /mirte_base_controller/cmd_vel_unstamped in simulation and /mirte_base_controller/cmd_vel on the real robot.'
        ),
        DeclareLaunchArgument('map', default_value=default_map_file),
        DeclareLaunchArgument('localization_params_file', default_value=default_localization_params),
        DeclareLaunchArgument('navigation_params_file', default_value=default_navigation_params),
        DeclareLaunchArgument('global_localization', default_value='false'),
        DeclareLaunchArgument('localization_bond_timeout', default_value='15.0'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_respawn', default_value='false'),
        DeclareLaunchArgument('log_level', default_value='info'),
        DeclareLaunchArgument('launch_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config_file', default_value=default_rviz_config),
        combined_launch,
    ])

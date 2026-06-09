from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
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

    autofocus = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'for dev in /dev/video*; do '
            'v4l2-ctl --device=$dev --set-ctrl=focus_automatic_continuous=1 2>/dev/null && echo "autofocus enabled on $dev" && continue; '
            'v4l2-ctl --device=$dev --set-ctrl=focus_auto=1 2>/dev/null && echo "autofocus enabled on $dev (legacy)"; '
            'done'
        ],
        output='screen',
        condition=UnlessCondition(use_sim_time),
    )

    wait_for_localization = ExecuteProcess(
        cmd=[
            PathJoinSubstitution([
                FindPackageShare('mdp_navigation'),
                'scripts',
                'wait_for_lifecycle_active.py'
            ]),
            '--nodes',
            '/map_server',
            '/amcl',
        ],
        output='screen',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_localization'),
                'launch',
                'localization.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': localization_params_file,
            'global_localization': global_localization,
            'bond_timeout': localization_bond_timeout,
            'use_rviz': 'false',
        }.items()
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_navigation'),
                'launch',
                'navigation.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'params_file': navigation_params_file,
            'odom_topic': odom_topic,
            'cmd_vel_topic': cmd_vel_topic,
            'autostart': autostart,
            'use_respawn': use_respawn,
            'log_level': log_level,
            'use_rviz': 'false',
        }.items()
    )

    rviz_node = Node(
        condition=IfCondition(launch_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    start_navigation_after_localization = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_localization,
            on_exit=[
                LogInfo(
                    msg='Localization is active; starting navigation lifecycle nodes.'
                ),
                navigation_launch,
            ],
        )
    )

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
    default_map_file = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'maps',
        'asym_map.yaml'
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
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
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map yaml file'
        ),
        DeclareLaunchArgument(
            'localization_params_file',
            default_value=default_localization_params,
            description='AMCL / localization parameters file'
        ),
        DeclareLaunchArgument(
            'navigation_params_file',
            default_value=default_navigation_params,
            description='Nav2 planner/controller parameters file'
        ),
        DeclareLaunchArgument(
            'global_localization',
            default_value='false',
            description='Run global localization after startup'
        ),
        DeclareLaunchArgument(
            'localization_bond_timeout',
            default_value='15.0',
            description='Seconds localization lifecycle manager waits for map_server/amcl bonds'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Autostart Nav2 lifecycle nodes'
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Respawn Nav2 nodes if they exit'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for Nav2 nodes'
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Launch RViz with the combined localization and navigation config'
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('mdp_navigation'),
                'rviz',
                'combined.rviz'
            ]),
            description='RViz config file to use for combined localization and navigation'
        ),
        autofocus,
        localization_launch,
        wait_for_localization,
        start_navigation_after_localization,
        rviz_node,
    ])

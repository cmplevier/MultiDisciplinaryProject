from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    global_localization = LaunchConfiguration('global_localization')
    use_rviz = LaunchConfiguration('use_rviz')
    autostart = LaunchConfiguration('autostart')
    bond_timeout = LaunchConfiguration('bond_timeout')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

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

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key='',
            param_rewrites={
                'use_sim_time': use_sim_time,
                'yaml_filename': map_file,
            },
            convert_types=True,
        ),
        allow_substs=True,
    )

    tf_remaps = [
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=tf_remaps,
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=tf_remaps,
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl'],
            'bond_timeout': bond_timeout,
        }],
    )

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
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
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically transition localization lifecycle nodes to active'
        ),
        DeclareLaunchArgument(
            'bond_timeout',
            default_value='15.0',
            description='Seconds lifecycle manager waits for map_server/amcl lifecycle bonds'
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='false',
            description='Respawn localization nodes if they exit'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for localization nodes'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz for localization'
        ),

        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        global_localization_cmd,
        rviz_cmd
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_file = LaunchConfiguration('map')
    plan_path = LaunchConfiguration('plan_path')
    clear_plan = LaunchConfiguration('clear_plan')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    longitudinal_margin_m = LaunchConfiguration('longitudinal_margin_m')
    lateral_offset_m = LaunchConfiguration('lateral_offset_m')
    click_search_radius_m = LaunchConfiguration('click_search_radius_m')
    occupied_threshold = LaunchConfiguration('occupied_threshold')

    default_map_file = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'maps',
        'asym_map.yaml',
    ])
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('mdp_navigation'),
        'rviz',
        'combined.rviz',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time while authoring tray waypoints',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Map used as the RViz background',
        ),
        DeclareLaunchArgument(
            'plan_path',
            default_value='~/mdp_ws/generated_row_plan.json',
            description='Writable JSON tray plan created by the generator',
        ),
        DeclareLaunchArgument(
            'clear_plan',
            default_value='false',
            description='Start from an empty generated plan',
        ),
        DeclareLaunchArgument(
            'longitudinal_margin_m',
            default_value='0.20',
            description='Extra waypoint distance beyond tray ends',
        ),
        DeclareLaunchArgument(
            'lateral_offset_m',
            default_value='0.35',
            description='Robot-center offset from each tray side',
        ),
        DeclareLaunchArgument(
            'click_search_radius_m',
            default_value='0.25',
            description='Search radius if the click misses an occupied pixel',
        ),
        DeclareLaunchArgument(
            'occupied_threshold',
            default_value='65',
            description='Map occupancy value treated as obstacle',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Open RViz for tray selection',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='RViz config used while authoring',
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='auto_tray_map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_auto_tray_authoring',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['auto_tray_map_server'],
            }],
        ),
        Node(
            package='mdp_mainloop',
            executable='auto_tray_waypoint_node',
            name='mdp_auto_tray_waypoint_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'plan_path': plan_path},
                {'clear_on_start': clear_plan},
                {'longitudinal_margin_m': longitudinal_margin_m},
                {'lateral_offset_m': lateral_offset_m},
                {'click_search_radius_m': click_search_radius_m},
                {'occupied_threshold': occupied_threshold},
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(launch_rviz),
        ),
    ])

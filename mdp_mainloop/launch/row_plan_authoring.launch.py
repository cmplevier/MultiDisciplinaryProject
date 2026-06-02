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
    seed_plan_path = LaunchConfiguration('seed_plan_path')
    clear_plan = LaunchConfiguration('clear_plan')
    launch_rviz = LaunchConfiguration('launch_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')

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
            description='Use simulation time while authoring row plans',
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Map used as the RViz background for row authoring',
        ),
        DeclareLaunchArgument(
            'plan_path',
            default_value='~/mdp_ws/generated_row_plan.json',
            description='Writable JSON file created by the row-plan builder',
        ),
        DeclareLaunchArgument(
            'seed_plan_path',
            default_value='',
            description='Optional existing JSON plan used when plan_path is absent',
        ),
        DeclareLaunchArgument(
            'clear_plan',
            default_value='false',
            description='Start from an empty generated plan',
        ),
        DeclareLaunchArgument(
            'launch_rviz',
            default_value='true',
            description='Open RViz with row-plan authoring tools',
        ),
        DeclareLaunchArgument(
            'rviz_config_file',
            default_value=default_rviz_config,
            description='RViz config containing row-plan tools and markers',
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='row_plan_map_server',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'yaml_filename': map_file},
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_row_plan_authoring',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['row_plan_map_server'],
            }],
        ),
        Node(
            package='mdp_mainloop',
            executable='row_plan_builder_node',
            name='mdp_row_plan_builder_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'plan_path': plan_path},
                {'seed_plan_path': seed_plan_path},
                {'clear_on_start': clear_plan},
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

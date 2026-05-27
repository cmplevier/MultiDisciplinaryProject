from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration('map')
    
    # Default Map Path
    default_map_file = PathJoinSubstitution([
        FindPackageShare('mdp_localization'),
        'maps',
        'asym_map.yaml'
    ])

    # 1. Include Gazebo Simulation Launch
    # This launches the greenhouse world, spawns the robot, and starts controllers + twist_mux
    gazebo_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_gazebo'),
                'launch',
                'greenhouse_world.launch.xml'
            ])
        ),
        launch_arguments={
            'rviz': 'false', # We will launch rviz through the navigation launch instead
        }.items()
    )

    # 2. Include Navigation & Localization Launch
    # We pass use_sim_time=true and route topics through the simulation-specific remaps
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_navigation'),
                'launch',
                'nav2_with_localization_and_rviz.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file,
            'odom_topic': '/mirte_base_controller/odom',
            'cmd_vel_topic': '/cmd_vel_nav', # Route through twist_mux
            'launch_rviz': 'true',
        }.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=default_map_file,
            description='Full path to map yaml file'
        ),
        
        gazebo_launch,
        nav_launch,
    ])

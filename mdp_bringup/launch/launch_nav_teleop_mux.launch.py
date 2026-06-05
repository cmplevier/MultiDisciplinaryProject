from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    # Launch Arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('mdp_localization'),
            'maps',
            'asym_map.yaml'
        ]),
        description='Full path to map yaml file to load'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # 1. SIMULATION MODE: Include sim_nav_loc_rviz (starts Gazebo)
    # In simulation, we use the Gazebo built-in twist_mux (configured in mirte-gazebo)
    nav_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_navigation'),
                'launch',
                'sim_nav_loc_rviz.launch.py'
            ])
        ),
        condition=IfCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'true',
            'cmd_vel_topic': '/cmd_vel_nav'
        }.items()
    )

    # 2. REAL ROBOT MODE: Include nav2_with_localization_and_rviz
    nav_real_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_navigation'),
                'launch',
                'nav2_with_localization_and_rviz.launch.py'
            ])
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': 'false',
            'cmd_vel_topic': 'cmd_vel_nav'
        }.items()
    )

    # 3. TWIST MUX (Only for Real Robot)
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_bringup'),
                'launch',
                'twist_mux.launch.py'
            ])
        ),
        condition=UnlessCondition(LaunchConfiguration('use_sim_time'))
    )

    # Include mdp_teleop teleop_custom launch file
    # We remap it to /cmd_vel_joy which is the joystick topic for both muxes
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('mdp_teleop'),
                'launch',
                'teleop_custom.launch.py'
            ])
        ),
        launch_arguments={
            'joy_config': 'custom_u22.yaml',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'cmd_vel_remap': '/cmd_vel_joy'
        }.items()
    )

    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        nav_sim_launch,
        nav_real_launch,
        twist_mux_launch,
        teleop_launch
    ])

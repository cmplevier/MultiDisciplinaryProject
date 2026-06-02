from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    clear_history = LaunchConfiguration('clear_history')
    row_plan_path = LaunchConfiguration('row_plan_path')
    generated_row_plan_path = LaunchConfiguration('generated_row_plan_path')
    planner_input_topic = LaunchConfiguration('planner_input_topic')
    row_plan_topic = LaunchConfiguration('row_plan_topic')
    auto_dispatch = LaunchConfiguration('auto_dispatch')
    return_home_after_rows = LaunchConfiguration('return_home_after_rows')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel_nav',
            description='Topic for velocity commands'),

        DeclareLaunchArgument(
            'clear_history',
            default_value='false',
            description='Clear mission history on startup'),

        DeclareLaunchArgument(
            'row_plan_path',
            default_value=PathJoinSubstitution([
                FindPackageShare('mdp_mainloop'),
                'config',
                'row_plan.json',
            ]),
            description='JSON file with discrete row approach/scan poses'),

        DeclareLaunchArgument(
            'generated_row_plan_path',
            default_value='~/mdp_ws/generated_row_plan.json',
            description='Finished JSON row plan consumed by the planner'),

        DeclareLaunchArgument(
            'planner_input_topic',
            default_value='/planner/row_scores',
            description='JSON topic used by planner to score/select rows'),

        DeclareLaunchArgument(
            'row_plan_topic',
            default_value='/planner/row_plan',
            description='Latched JSON topic with the active row plan'),

        DeclareLaunchArgument(
            'auto_dispatch',
            default_value='true',
            description='Let the planner dispatch tasks automatically'),

        DeclareLaunchArgument(
            'return_home_after_rows',
            default_value='true',
            description='Send a final return_home NAV_ONLY task'),

        Node(
            package='mdp_mainloop',
            executable='high_level_planner_node',
            name='mdp_high_level_planner_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'clear_history': clear_history},
                {'row_plan_path': row_plan_path},
                {'generated_row_plan_path': generated_row_plan_path},
                {'require_generated_row_plan': True},
                {'planner_input_topic': planner_input_topic},
                {'row_plan_topic': row_plan_topic},
                {'auto_dispatch': auto_dispatch},
                {'return_home_after_rows': return_home_after_rows},
            ]
        ),

        Node(
            package='mdp_mainloop',
            executable='mainloop_node',
            name='mdp_mainloop_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'cmd_vel_topic': cmd_vel_topic},
            ]
        )
    ])

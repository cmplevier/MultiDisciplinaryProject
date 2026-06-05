from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    clear_history = LaunchConfiguration('clear_history')
    plan_path = LaunchConfiguration('plan_path')
    load_plan_file = LaunchConfiguration('load_plan_file')
    require_plan_file = LaunchConfiguration('require_plan_file')
    loop_mission = LaunchConfiguration('loop_mission')
    task_topic = LaunchConfiguration('task_topic')
    strafe_costmap_enabled = LaunchConfiguration('strafe_costmap_enabled')
    strafe_costmap_topic = LaunchConfiguration('strafe_costmap_topic')
    strafe_require_costmap = LaunchConfiguration('strafe_require_costmap')
    strafe_block_unknown_costmap = LaunchConfiguration(
        'strafe_block_unknown_costmap'
    )
    strafe_block_timeout_sec = LaunchConfiguration(
        'strafe_block_timeout_sec'
    )
    blocked_tray_selection_mode = LaunchConfiguration(
        'blocked_tray_selection_mode'
    )
    blocked_tray_retry_delay_sec = LaunchConfiguration(
        'blocked_tray_retry_delay_sec'
    )

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
            'plan_path',
            default_value='~/mdp_ws/generated_row_plan.json',
            description='JSON row plan to execute in order'),

        DeclareLaunchArgument(
            'load_plan_file',
            default_value='true',
            description=(
                'Load plan_path directly instead of only waiting for tasks'
            )),

        DeclareLaunchArgument(
            'require_plan_file',
            default_value='false',
            description='Log an error if plan_path is missing'),

        DeclareLaunchArgument(
            'loop_mission',
            default_value='false',
            description='Restart the JSON plan after all tasks finish'),

        DeclareLaunchArgument(
            'task_topic',
            default_value='/planner/next_task',
            description='JSON task topic used when load_plan_file is false'),

        DeclareLaunchArgument(
            'strafe_costmap_enabled',
            default_value='true',
            description='Use the local costmap to filter strafe commands'),

        DeclareLaunchArgument(
            'strafe_costmap_topic',
            default_value='/local_costmap/costmap',
            description='OccupancyGrid topic used for strafe safety checks'),

        DeclareLaunchArgument(
            'strafe_require_costmap',
            default_value='false',
            description='Stop strafing if the local costmap is unavailable'),

        DeclareLaunchArgument(
            'strafe_block_unknown_costmap',
            default_value='true',
            description='Treat unknown local-costmap cells as blocked'),

        DeclareLaunchArgument(
            'strafe_block_timeout_sec',
            default_value='8.0',
            description='Seconds to wait before skipping a blocked tray'),

        DeclareLaunchArgument(
            'blocked_tray_selection_mode',
            default_value='random_tray',
            description='Selection after blocked strafe: random_tray or ordered'),

        DeclareLaunchArgument(
            'blocked_tray_retry_delay_sec',
            default_value='60.0',
            description='Seconds before a skipped tray can be retried'),

        Node(
            package='mdp_mainloop',
            executable='mainloop_node',
            name='mdp_mainloop_node',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'cmd_vel_topic': cmd_vel_topic},
                {'clear_history': clear_history},
                {'plan_path': plan_path},
                {'load_plan_file': load_plan_file},
                {'require_plan_file': require_plan_file},
                {'loop_mission': loop_mission},
                {'task_topic': task_topic},
                {'strafe_costmap_enabled': strafe_costmap_enabled},
                {'strafe_costmap_topic': strafe_costmap_topic},
                {'strafe_require_costmap': strafe_require_costmap},
                {'strafe_block_unknown_costmap': strafe_block_unknown_costmap},
                {'strafe_block_timeout_sec': strafe_block_timeout_sec},
                {'blocked_tray_selection_mode': blocked_tray_selection_mode},
                {'blocked_tray_retry_delay_sec': blocked_tray_retry_delay_sec},
            ]
        )
    ])

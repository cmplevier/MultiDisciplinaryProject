from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile

def generate_launch_description():

    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    log_level = LaunchConfiguration("log_level")
    map_yaml_file = LaunchConfiguration("map")
    use_rviz = LaunchConfiguration("use_rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("mdp_navigation"), "rviz", "navigation.rviz"]
    )

    configured_params = ParameterFile(
        params_file,
        allow_substs=True,
    )

    nav_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    tf_remaps = [
        ("/tf", "tf"),
        ("/tf_static", "tf_static"),
    ]

    cmd_vel_remap = [("cmd_vel", "/cmd_vel_nav")]

    nav_actions = [
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace,
        ),

        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=tf_remaps + cmd_vel_remap,
        ),

        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=tf_remaps + cmd_vel_remap,
        ),

        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=tf_remaps + cmd_vel_remap,
        ),

        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            output="screen",
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params],
            arguments=["--ros-args", "--log-level", log_level],
            remappings=tf_remaps + cmd_vel_remap,
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            arguments=["--ros-args", "--log-level", log_level],
            parameters=[
                {
                    "autostart": autostart,
                    "node_names": nav_nodes,
                    "use_sim_time": use_sim_time,
                },
            ],
        ),

        Node(
            condition=IfCondition(use_rviz),
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
            parameters=[{"use_sim_time": use_sim_time}],
            remappings=tf_remaps,
        ),
    ]

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Top-level namespace for Nav2 nodes.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_namespace",
            default_value="False",
            description="Whether to apply the namespace to Nav2 nodes.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="True",
            description="Use Gazebo simulation time.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "params_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mdp_navigation"), "config", "nav2_params.yaml"]
            ),
            description="Full path to the Nav2 parameters file.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "map",
            default_value=PathJoinSubstitution(
                [FindPackageShare("mdp_localization"), "maps", "test_map.yaml"]
            ),
            description="Full path to map yaml file to load",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "autostart",
            default_value="True",
            description="Automatically transition Nav2 lifecycle nodes to active.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_respawn",
            default_value="False",
            description="Respawn Nav2 nodes if one exits.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            description="Logging level for Nav2 nodes.",
        )
    )

    ld.add_action(
        DeclareLaunchArgument(
            "use_rviz",
            default_value="True",
            description="Whether to start RViz",
        )
    )

    ld.add_action(GroupAction(nav_actions))

    return ld

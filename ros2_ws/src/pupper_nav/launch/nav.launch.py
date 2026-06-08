"""Lite AMCL + Nav2 launch for Pupper v3.

Prerequisites:
  - robot.service or neural_controller launch running with odom_ekf:=True
  - LD06 publishing /scan
  - saved map YAML available on the robot
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    map_yaml = LaunchConfiguration("map")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_map = DeclareLaunchArgument(
        "map",
        default_value="/home/pi/maps/pupper_map_ekf_v1.yaml",
        description="Full path to the map yaml to load.",
    )
    declare_params = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [FindPackageShare("pupper_nav"), "config", "nav2_params.yaml"]
        ),
        description="Full path to the Nav2 parameter file.",
    )
    declare_autostart = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically configure and activate lifecycle nodes.",
    )
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation time.",
    )

    common_params = [
        params_file,
        {
            "use_sim_time": use_sim_time,
            "bond_heartbeat_period": 0.0,
        },
    ]

    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            params_file,
            {
                "yaml_filename": map_yaml,
                "use_sim_time": use_sim_time,
                "bond_heartbeat_period": 0.0,
            },
        ],
    )

    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=common_params,
    )

    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=common_params,
        remappings=[("cmd_vel", "/nav_cmd_vel")],
    )

    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=common_params,
    )

    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=common_params,
    )

    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=common_params,
    )

    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=common_params,
    )

    lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "autostart": autostart,
                "node_names": lifecycle_nodes,
                "bond_timeout": 0.0,
            }
        ],
    )

    return LaunchDescription(
        [
            declare_map,
            declare_params,
            declare_autostart,
            declare_use_sim_time,
            map_server,
            amcl,
            controller_server,
            smoother_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager,
        ]
    )

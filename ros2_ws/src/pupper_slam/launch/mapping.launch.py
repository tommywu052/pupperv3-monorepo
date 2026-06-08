"""Full mapping stack: optional LD06 + slam_toolbox.

Usage on Pi (robot.service should already be running):
  source /opt/ros/jazzy/setup.bash
  source ~/ldlidar_ros2_ws/install/setup.bash
  source ~/pupperv3-monorepo/ros2_ws/install/setup.bash
  ros2 launch pupper_slam mapping.launch.py

Foxglove: set Fixed Frame to `map`, add Map display on /map.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declare_lidar = DeclareLaunchArgument(
        "lidar",
        default_value="true",
        description="Start LD06 driver (requires ldlidar_stl_ros2 in workspace overlay).",
    )

    ld06_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ldlidar_stl_ros2"),
                    "launch",
                    "ld06.launch.py",
                ]
            )
        ),
        condition=IfCondition(LaunchConfiguration("lidar")),
    )

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("pupper_slam"), "launch", "slam.launch.py"]
            )
        ),
    )

    return LaunchDescription([declare_lidar, ld06_launch, slam_launch])

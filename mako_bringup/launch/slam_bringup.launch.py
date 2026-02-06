#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_mako_description = get_package_share_directory("mako_description")
    pkg_mako_navigation_path = get_package_share_directory("mako_navigation")

    mako_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("mako_gazebo"),
                "launch",
                "robot_gz.launch.py",
            ]
        ),
    )

    slam_params = os.path.join(pkg_mako_navigation_path, "config", "slam_params.yaml")
    rviz_config = os.path.join(pkg_mako_description, "rviz", "slam-toolbox.yaml.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )

    # === SLAM Toolbox ===
    slam_toolbox = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": use_sim_time}],
    )

    # === RViz ===
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            mako_gazebo,
            slam_toolbox,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=slam_toolbox,
                    on_exit=[mako_gazebo],
                )
            ),
            rviz,
        ]
    )

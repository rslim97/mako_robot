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

    mako_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("mako_gazebo"),
                "launch",
                "robot_gz.launch.py",
            ]
        ),
    )

    # lio_sam = IncludeLaunchDescription(
    #     PathJoinSubstitution(
    #         [
    #             FindPackageShare("lio_sam"),
    #             "launch",
    #             "run.launch.py",
    #         ]
    #     ),
    # )

    scanmatcher_lio = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("scanmatcher"),
                "launch",
                "lio.launch.py",
            ]
        ),
    )

    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )

    return LaunchDescription(
        [
            declare_use_sim_time,
            mako_gazebo,
            scanmatcher_lio,
            # RegisterEventHandler(
            #     event_handler=OnProcessExit(
            #         target_action=lio_sam,
            #         on_exit=[mako_gazebo],
            #     )
            # ),
            # rviz,
            # joy_node,
            # teleop_joy,
        ]
    )

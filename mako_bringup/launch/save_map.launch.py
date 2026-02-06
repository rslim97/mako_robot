#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

# from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
# from launch_ros.substitutions import FindPackageShare
# import os


def generate_launch_description():
    # proj_root = FindPackageShare("mecanum_bringup")
    # map_file = PathJoinSubstitution([proj_root, "mecanum_navigation", "maps", "mapa"])
    # map_file_name = os.path.join(map_location, "mapa")
    map_file = "./src/mako_navigation/maps/mapa"
    return LaunchDescription(
        [
            Node(
                package="nav2_map_server",
                executable="map_saver_cli",
                name="map_saver",
                output="screen",
                parameters=[{"use_sim_time": True}],
                arguments=[
                    "-f",
                    map_file,
                ],
            )
        ]
    )

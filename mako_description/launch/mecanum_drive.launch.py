# Copyright 2024 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("mako_description"),
                    "urdf",
                    "model",
                    "mecanum_drive.xacro.urdf",
                ]
            ),
        ]
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("mako_description"),
            "config",
            "mecanum_drive_controller.yaml",
        ]
    )

    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description_content,
            }
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )
    mecanum_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "mecanum_drive_controller",
            "--param-file",
            robot_controllers,
        ],
    )

    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessStart(
                    target_action=node_robot_state_publisher,
                    on_start=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=joint_state_broadcaster_spawner,
                    on_exit=[mecanum_drive_controller_spawner],
                )
            ),
            node_robot_state_publisher,
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="If true, use simulated clock",
            ),
        ]
    )

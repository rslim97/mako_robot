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
    # === Paquetes ===
    # pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    # pkg_axioma_description = get_package_share_directory("axioma_description")
    # pkg_mecanum_gazebo = get_package_share_directory("mecanum_gazebo")
    # pkg_mecanum_gazebo_path = get_package_share_directory("mecanum_gazebo")
    pkg_mecanum_description = get_package_share_directory("mecanum_description")
    pkg_mecanum_navigation_path = get_package_share_directory("mecanum_navigation")

    mecanum_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("mecanum_gazebo"),
                "launch",
                "robot_gz.launch.py",
            ]
        ),
    )

    # === Archivos ===
    # world = os.path.join(pkg_axioma_description, "worlds", "empty.world")
    # sdf_model = os.path.join(pkg_axioma_description, "models", "axioma_v2", "model.sdf")
    slam_params = os.path.join(
        pkg_mecanum_navigation_path, "config", "slam_params.yaml"
    )
    rviz_config = os.path.join(
        pkg_mecanum_description, "rviz", "slam-toolbox.yaml.rviz"
    )
    # urdf_file = os.path.join(pkg_axioma_description, "urdf", "axioma.urdf")

    # === Argumentos ===
    use_sim_time = LaunchConfiguration("use_sim_time")
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation clock"
    )

    # # === Gazebo ===
    # gzserver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
    #     ),
    #     launch_arguments={'world': world}.items(),
    # )
    # gzclient = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
    #     )
    # )

    # # === Spawn del robot ===
    # spawn_robot = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     arguments=[
    #         '-entity', 'axioma',
    #         '-file', sdf_model,
    #         '-x', '0.0', '-y', '0.0', '-z', '0.1'
    #     ],
    #     output='screen'
    # )

    # # === Robot State Publisher ===
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}]
    # )

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

    # # === Nodo del joystick (control Xbox) ===
    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     name="joy_node",
    #     output="screen",
    #     parameters=[{"dev": "/dev/input/js0"}],
    # )

    # # === Teleop Twist Joy (traduce joystick → cmd_vel) ===
    # teleop_joy = Node(
    #     package="teleop_twist_joy",
    #     executable="teleop_node",
    #     name="teleop_twist_joy",
    #     output="screen",
    #     parameters=[
    #         {
    #             "use_sim_time": use_sim_time,
    #             "axis_linear.x": 1,  # eje izquierdo vertical
    #             "axis_angular.yaw": 0,  # eje derecho horizontal
    #             "scale_linear.x": 0.5,  # velocidad lineal máxima
    #             "scale_angular.yaw": 2.0,  # velocidad angular máxima
    #         }
    #     ],
    # )

    # === Launch final ===
    return LaunchDescription(
        [
            declare_use_sim_time,
            mecanum_gazebo,
            # gzserver,
            # gzclient,
            # spawn_robot,
            # robot_state_publisher,
            slam_toolbox,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=slam_toolbox,
                    on_exit=[mecanum_gazebo],
                )
            ),
            rviz,
            # joy_node,
            # teleop_joy,
        ]
    )

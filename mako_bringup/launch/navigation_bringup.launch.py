#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # pkg_gazebo_ros = get_package_share_directory("gazebo_ros")
    pkg_mako_description_path = get_package_share_directory("mako_description")
    pkg_mako_navigation_path = get_package_share_directory("mako_navigation")

    map_file = "./src/mako_navigation/maps/mapa.yaml"
    nav2_params_file = os.path.join(
        pkg_mako_navigation_path, "config", "nav2_params.yaml"
    )
    ekf_params_file = os.path.join(pkg_mako_navigation_path, "config", "ekf.yaml")

    mako_gazebo = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("mako_gazebo"),
                "launch",
                "robot_gz.launch.py",
            ]
        ),
    )
    rviz_config = os.path.join(
        pkg_mako_description_path, "rviz", "navigation.yaml.rviz"
    )

    robot_localization_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_params_file, {"use_sim_time": True}],
        # remappings=[("odometry/filtered", LaunchConfiguration(""))],
    )

    # # Gazebo server
    # gzserver = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, "launch", "gzserver.launch.py")
    #     ),
    #     launch_arguments={"world": world}.items(),
    # )

    # # Gazebo client
    # gzclient = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(pkg_gazebo_ros, "launch", "gzclient.launch.py")
    #     )
    # )

    # # Spawn robot - delayed to ensure Gazebo is ready
    # spawn_robot = TimerAction(
    #     period=3.0,
    #     actions=[
    #         Node(
    #             package="gazebo_ros",
    #             executable="spawn_entity.py",
    #             arguments=[
    #                 "-entity",
    #                 "axioma",
    #                 "-file",
    #                 sdf_model,
    #                 "-x",
    #                 "0.0",
    #                 "-y",
    #                 "0.0",
    #                 "-z",
    #                 "0.1",
    #             ],
    #             output="screen",
    #         )
    #     ],
    # )

    # # Robot state publisher
    # with open(urdf_file, "r") as infp:
    #     robot_desc = infp.read()

    # robot_state_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_state_publisher",
    #     output="screen",
    #     parameters=[{"use_sim_time": True, "robot_description": robot_desc}],
    # )

    # Map server
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"use_sim_time": True, "yaml_filename": map_file}],
    )

    # Lifecycle manager for map_server and amcl
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }
        ],
    )

    # AMCL
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[nav2_params_file],
    )

    # Controller server
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        output="screen",
        parameters=[nav2_params_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Planner server
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[nav2_params_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Behavior server
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[nav2_params_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # BT Navigator
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[nav2_params_file],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
    )

    # Lifecycle manager for navigation
    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "autostart": True,
                "node_names": [
                    "controller_server",
                    "planner_server",
                    "behavior_server",
                    "bt_navigator",
                ],
            }
        ],
    )

    # # Static transform publisher for initial map->odom (temporary, until AMCL initializes)
    # static_transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     name="static_map_to_odom",
    #     arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
    # )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    return LaunchDescription(
        [
            # gzserver,
            # gzclient,
            # spawn_robot,
            # robot_state_publisher,
            mako_gazebo,
            robot_localization_node,
            # static_transform_publisher,
            map_server,
            amcl,
            lifecycle_manager_localization,
            controller_server,
            planner_server,
            behavior_server,
            bt_navigator,
            lifecycle_manager_navigation,
            rviz,
        ]
    )

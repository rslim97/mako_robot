import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.substitutions import (
    LaunchConfiguration,
    PythonExpression,
    PathJoinSubstitution,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    mecanum_gazebo_path = get_package_share_directory("mecanum_gazebo")
    # mecanum_control_path = get_package_share_directory("mecanum_control")
    world_file = LaunchConfiguration(
        "world_file",
        default=os.path.join(mecanum_gazebo_path, "worlds", "small_house.sdf"),
    )
    gz_sim_share = get_package_share_directory("ros_gz_sim")
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": PythonExpression(["'", world_file, " -r -v 4 '"])
        }.items(),
    )

    """ Mecanum Drive """

    # mecanum_description_path = PathJoinSubstitution(
    #     [
    #         FindPackageShare("mecanum_description"),
    #         "launch",
    #         "mecanum_drive_example.launch.py",
    #     ]
    # )

    # robot_and_joint_state = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(mecanum_description_path),
    #     launch_arguments={"use_sim_time": use_sim_time}.items(),
    # )

    """ Differential Drive """

    diff_drive_description_path = PathJoinSubstitution(
        [
            FindPackageShare("mecanum_description"),
            "launch",
            "diff_drive_example.launch.py",
        ]
    )

    robot_and_joint_state = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diff_drive_description_path),
        launch_arguments={"use_sim_time": "true"}.items(),
    )

    """ Spawn robot in gz """
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "groundhog",
            "-topic",
            "/robot_description",
            "-x",
            "0",
            "-y",
            "0",
            "-z",
            "1.4",
        ],
        output="screen",
    )

    # # Launch the Gazebo-ROS bridge
    bridge_params = os.path.join(mecanum_gazebo_path, "config", "gz_bridge.yaml")
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "--ros-args",
            "-p",
            f"config_file:={bridge_params}",
        ],
    )

    return LaunchDescription(
        [
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.join(mecanum_gazebo_path, "worlds"),
            ),
            AppendEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=os.path.join(mecanum_gazebo_path, "models"),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("world_file", default_value=world_file),
            gz_sim,
            ros_gz_bridge,
            spawn_entity,
            robot_and_joint_state,
        ]
    )

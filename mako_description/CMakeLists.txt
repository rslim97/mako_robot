cmake_minimum_required(VERSION 3.5)
project(mako_description)

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(
  DIRECTORY launch config urdf rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

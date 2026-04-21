cmake_minimum_required(VERSION 3.8)
project(me_robot_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    urdf/
  DESTINATION share/${PROJECT_NAME}/urdf
)

install(
  DIRECTORY
    rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

ament_package()

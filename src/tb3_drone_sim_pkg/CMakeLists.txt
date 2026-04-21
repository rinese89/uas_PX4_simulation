cmake_minimum_required(VERSION 3.8)
project(tb3_drone_sim_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY rviz
DESTINATION share/${PROJECT_NAME})

ament_package()

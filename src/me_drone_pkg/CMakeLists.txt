cmake_minimum_required(VERSION 3.8)
project(me_drone_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(std_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(geometry_msgs)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
#find_package(px4_ros_com REQUIRED)


add_executable(lidar2odom src/lidar2odom.cpp)
ament_target_dependencies(lidar2odom rclcpp sensor_msgs geometry_msgs tf2 tf2_ros tf2_sensor_msgs laser_geometry)

add_executable(offboard_control src/offboard_control.cpp)
ament_target_dependencies(offboard_control rclcpp px4_msgs std_msgs std_srvs)

add_executable(cmd_vel_node src/cmd_vel_node.cpp)
ament_target_dependencies(cmd_vel_node rclcpp px4_msgs std_msgs std_srvs)

add_executable(drone_odom_broadcaster src/drone_odom_broadcaster.cpp)
ament_target_dependencies(drone_odom_broadcaster rclcpp px4_msgs tf2_msgs tf2_ros geometry_msgs tf2 nav_msgs std_srvs Eigen3)

add_executable(mision_node src/mision_node.cpp)
ament_target_dependencies(mision_node rclcpp px4_msgs tf2_msgs tf2_ros geometry_msgs tf2 nav_msgs std_srvs visualization_msgs)

add_executable(drone_odom_master_broadcaster src/drone_odom_master_broadcaster.cpp)
ament_target_dependencies(drone_odom_master_broadcaster rclcpp px4_msgs tf2_msgs tf2_ros geometry_msgs tf2 nav_msgs)

install(TARGETS
  offboard_control
  cmd_vel_node
  drone_odom_master_broadcaster
  drone_odom_broadcaster
  mision_node
  lidar2odom
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY urdf
DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME})

ament_package()

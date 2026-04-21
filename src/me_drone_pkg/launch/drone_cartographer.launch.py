"""
  Copyright 2018 The Cartographer Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    ## ***** File paths ******
    pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    urdf_dir = '/home/rinese/robot_ws/src/me_drone_pkg/urdf'
    urdf_file = os.path.join(urdf_dir, 'x500_base.urdf')

    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Ruta de tu configuración LUA
    my_config_dir = '/home/rinese/robot_ws/src/me_drone_pkg/config'
    my_config_file = 'drone_lidar_config.lua'

    # Ruta RViz2
    rviz_dir = '/home/rinese/robot_ws/src/me_drone_pkg/rviz'
    rviz_file = 'drone_cartographer.rviz'

    ## ***** Nodes *****
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_desc},
            {'use_sim_time': True}
        ],
        output='screen'
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', my_config_dir,
            '-configuration_basename', my_config_file
        ],
        output='screen'
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        parameters=[
            {'use_sim_time': True},
            {'resolution': 0.05}
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        on_exit=Shutdown(),
        arguments=[
            '-d',
            os.path.join(rviz_dir, rviz_file)
        ],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        cartographer_node,
        cartographer_occupancy_grid_node,
        rviz_node,
    ])
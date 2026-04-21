import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node


def generate_launch_description():
    # Configure ROS nodes for launch

    # Setup project paths
    pkg_project = get_package_share_directory('me_robot_pkg')

    # Load the SDF file from "description" package
    urdf_file  =  os.path.join(pkg_project, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Takes the description and joint angles as inputs and publishes the 3D poses of the robot links
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ],
        #remappings=[
        #('/robot_description', '/mebot1/robot_description'),
        #('/joint_states','/mebot1/joint_states'),
        #]
        )

    #diff_drive_controller = Node(
    #package="controller_manager",
    #executable="spawner",
    #arguments=["diff_drive_controller"],
    #output="screen"
    #)

    #join_state_broadcaster = Node(
    #package="controller_manager",
    #executable="spawner",
    #arguments=["join_state_broadcaster"],
    #output="screen"
    #)

    Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock"],
        output="screen"
    )

    # Visualize in RViz
    rviz = Node(
       package='rviz2',
       executable='rviz2',
       arguments=['-d', os.path.join(pkg_project, 'rviz', 'meRobot.rviz')],
       condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true',
                              description='Open RViz.'),
        robot_state_publisher,
        #diff_drive_controller,
        #join_state_broadcaster,
        #bridge_ros_gz,
        rviz
    ])
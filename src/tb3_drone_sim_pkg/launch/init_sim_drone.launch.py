import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, GroupAction
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    tb3_drone_dir = get_package_share_directory('tb3_drone_sim_pkg')

    drone_dir = get_package_share_directory('me_drone_pkg')

    px4_dir = '/home/rinese/PX4-Autopilot'

     # Load the SDF file from "description" package
    urdf_file  =  os.path.join(drone_dir, 'urdf', 'x500_base.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    # Nodo del robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc}]
    )

    actions=[

        ExecuteProcess(
            cmd=[['/home/rinese/QGroundControl-x86_64.AppImage']],
            output='screen'
        ),

        #ExecuteProcess(
        #    cmd=['make', 'px4_sitl_default', 'gz_x500_mono_cam'],
        #    cwd=px4_dir,
        #    output='screen'
        #),

        ExecuteProcess(
            cmd=[
                'bash', '-c',
                'PX4_GZ_MODEL_POSE="0,0,0,0,0,0" make px4_sitl_default gz_x500_mono_cam'
            ],
            cwd=px4_dir,
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_parameter_bridge',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'config_file': '/home/rinese/robot_ws/src/tb3_drone_sim_pkg/config/bridge_config_drone.yaml'}]
        ),

        # Visualize in RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(tb3_drone_dir, 'rviz', 'sim_drone.rviz')],
            parameters=[
            {'use_sim_time': True}]        
        ),

        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=['udp4', '--port', '8888'],
            output='screen'
        ),

        Node(
            package='me_drone_pkg',
            executable='drone_odom_broadcaster',
            name='drone_odom_broadcaster',
            output='screen',
            parameters=[
            {'use_sim_time': True}]
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0',   # x y z
                       '1.5707963',     # yaw 90° en radianes
                       '0', '0',        # pitch roll
                       'map', 'odom'],
            parameters=[{'use_sim_time': True}]
        ),

        #Node(
        #    package='me_drone_pkg',
        #    executable='lidar2odom',
        #    name='lidar_to_odom',
        #    output='screen',
        #    parameters=[
        #        {'use_sim_time': True},
        #        {'source_frame': 'x500_mono_cam_0/lidar_link/lidar_2d'},
        #        {'target_frame': 'odom'},
        #        {'input_topic':  '/scan'},
        #        {'output_topic': '/scan_odom'},
        #    ]
        #),

        robot_state_publisher_node

    ]

    tb3_drone_group = GroupAction(actions)

    ld= LaunchDescription()
    ld.add_action(tb3_drone_group)

    return ld
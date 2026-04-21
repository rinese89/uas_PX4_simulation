import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess, GroupAction
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    #drone_dir = LaunchConfiguration(
    #    'drone_dir',
    #    default=os.path.join(
    #        get_package_share_directory('me_drone_pkg','launch')
    #    )
    #)

    # Get the pkg name dinamically

    #tb3_dir = LaunchConfiguration(
    #    'tb3_dir',
    #    default=os.path.join(
    #        get_package_share_directory('me_robot_pkg','launch')
    #    )
    #)

    # Get the pkg name

    tb3_dir = get_package_share_directory('me_robot_pkg')

    drone_dir = get_package_share_directory('me_drone_pkg')

    # Load the SDF file from "description" package
    urdf_file  =  os.path.join(tb3_dir, 'urdf', 'turtlebot3_waffle_pi.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    px4_dir = '/home/rinese/drone_px4_ws/src/PX4-Autopilot'

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

    # Spawner para diff_drive
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller"],
        output="screen"
    )

    #Spawner para joint_state_broadcaster (si decides activarlo)
    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen"
    )

    static_tf_odom_drone_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_drone',
        arguments=['2', '0', '0.1', '0', '0', '0', '1', 'odom_drone', 'odom'],
        output='screen'
    )

    delayed_controller_spawners = RegisterEventHandler(
    OnProcessStart(
        target_action=robot_state_publisher_node,
        on_start=[
            diff_drive_spawner,
            joint_state_spawner
        ]
    )
    )

    #delayed_controller_spawners = TimerAction(
    #    period=5.0,  # espera 5 segundos antes de lanzar los spawners
    #    actions=[
    #       static_tf_odom_drone_node
    #    ]
    #    )

    actions=[

        robot_state_publisher_node,

        ExecuteProcess(
            cmd=[['/home/rinese/tb3_drone_ws/src/tb3_drone_sim_pkg/utils/QGroundControl-x86_64.AppImage']],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['make', 'px4_sitl_default', 'gz_x500'],
            cwd=px4_dir,
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='gz_parameter_bridge',
            output='screen',
            parameters=[{'config_file': '/home/rinese/tb3_drone_ws/src/tb3_drone_sim_pkg/config/gz_msg_bridge_tb3_drone.yaml'}]
        ),

        # Visualize in RViz
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(tb3_dir, 'rviz', 'meRobot.rviz')],
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
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_odom_drone',
            arguments=['2', '0', '0.1', '0', '0', '0', '1', 'odom_drone', 'odom'],
            output='screen'
        ),

        delayed_controller_spawners,

    ]

    tb3_drone_group = GroupAction(actions)

    ld= LaunchDescription()
    ld.add_action(tb3_drone_group)

    return ld
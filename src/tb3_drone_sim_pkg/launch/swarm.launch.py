from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    topics = [
        "/fmu/in/vehicle_command",
        "/fmu/in/offboard_control_mode",
        "/fmu/in/trajectory_setpoint",
    ]

    nodes = []

    # =========================
    # RELAYS (fan-out PX4)
    # =========================
    for topic in topics:
        nodes.append(
            Node(
                package='topic_tools',
                executable='relay',
                name=f"relay_px4_1_{topic.replace('/', '_')}",
                arguments=[topic, f"/px4_1{topic}"],
                output='screen'
            )
        )

        nodes.append(
            Node(
                package='topic_tools',
                executable='relay',
                name=f"relay_px4_2_{topic.replace('/', '_')}",
                arguments=[topic, f"/px4_2{topic}"],
                output='screen'
            )
        )

    # =========================
    # 🚁 PX4 SITL instancia 1
    # =========================
    px4_1 = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'PX4_SYS_AUTOSTART=4001 '
            'PX4_SIM_MODEL=gz_x500 '
            'PX4_GZ_MODEL_POSE="0,0,0,0,0,0" '
            './build/px4_sitl_default/bin/px4 -i 1'
        ],
        cwd='/home/rinese/PX4-Autopilot', 
        output='screen'
    )

    # =========================
    # PX4 SITL instancia 2
    # =========================
    px4_2 = ExecuteProcess(
        cmd=[
            'bash', '-c',
            'PX4_SYS_AUTOSTART=4001 '
            'PX4_SIM_MODEL=gz_x500 '
            'PX4_GZ_MODEL_POSE="2,0,0,0,0,0" '
            './build/px4_sitl_default/bin/px4 -i 2'
        ],
        cwd='/home/rinese/tuPX4-Autopilot', 
        output='screen'
    )

    # =========================
    # micro-ROS agent
    # =========================
    micro_ros = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],
        output='screen'
    )

    # =========================
    # Launch final
    # =========================
    return LaunchDescription(
        nodes + [
            px4_1,
            px4_2,
            micro_ros
        ]
    )
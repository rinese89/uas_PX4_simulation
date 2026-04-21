from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_offboard_nodes(context):

    num_drones = int(LaunchConfiguration('num_drones').perform(context))

    nodes = []

    # =========================
    # 🚀 MASTER DRONE (sin namespace)
    # =========================
    nodes.append(
        Node(
            package='me_drone_pkg',
            executable='offboard_control',
            name='offboard_control_master',
            output='screen',
            parameters=[{
                'drone_ns': "",          # 👈 sin namespace
                'target_system': 1       # 👈 MASTER
            }]
        )
    )

    # =========================
    # 🚀 DRONES CON NAMESPACE
    # =========================
    for i in range(1, num_drones + 1):

        drone_ns = f'px4_{i}'
        target_system = i + 1

        nodes.append(
            Node(
                package='me_drone_pkg',
                executable='offboard_control',
                name=f'offboard_control_{i}',
                output='screen',
                parameters=[{
                    'drone_ns': drone_ns,
                    'target_system': target_system
                }]
            )
        )

    return nodes


def generate_launch_description():

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='0',
        description='Numero de drones secundarios (px4_i)'
    )

    offboard_nodes = OpaqueFunction(function=generate_offboard_nodes)

    # Delay → esperar a PX4 + micro-ROS
    delayed_offboard = TimerAction(
        period=4.0,
        actions=[offboard_nodes]
    )

    return LaunchDescription([
        num_drones_arg,
        delayed_offboard
    ])
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_mission_nodes(context):

    num_drones = int(LaunchConfiguration('num_drones').perform(context))

    nodes = []

    # =========================
    # 🚀 MASTER DRONE (sin namespace)
    # =========================
    nodes.append(
        Node(
            package='me_drone_pkg',
            executable='mision_node',
            name='mision_node_master',
            output='screen',
            parameters=[{
                'drone_ns': "",
                'use_sim_time': True
            }]
        )
    )

    # =========================
    # 🚀 DRONES CON NAMESPACE
    # =========================
    for i in range(1, num_drones + 1):

        drone_ns = f'px4_{i}'

        nodes.append(
            Node(
                package='me_drone_pkg',
                executable='mision_node',
                name=f'mision_node_{i}',
                output='screen',
                parameters=[{
                    'drone_ns': drone_ns,
                    'use_sim_time': True
                }]
            )
        )

    return nodes


def generate_launch_description():

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='2',
        description='Numero de drones secundarios (px4_i)'
    )

    mission_nodes = OpaqueFunction(function=generate_mission_nodes)

    delayed_mission = TimerAction(
        period=4.0,
        actions=[mission_nodes]
    )

    return LaunchDescription([
        num_drones_arg,
        delayed_mission
    ])
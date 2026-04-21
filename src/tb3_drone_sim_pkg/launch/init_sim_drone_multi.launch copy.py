from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration


def generate_px4_processes(context):

    px4_dir = '/home/rinese/PX4-Autopilot'
    num_drones = int(LaunchConfiguration('num_drones').perform(context))

    actions = []

    for i in range(1, num_drones + 1):

        x_pos = 2.0 * i

        cmd = (
            f'PX4_SYS_AUTOSTART=4001 '
            f'PX4_SIM_MODEL=gz_x500 '
            f'PX4_GZ_MODEL_POSE="{x_pos},0,0,0,0,0" '
            f'./build/px4_sitl_default/bin/px4 -i {i}'
        )

        actions.append(
            ExecuteProcess(
                cmd=['bash', '-c', cmd],
                cwd=px4_dir,
                output='screen'
            )
        )

    return actions


def generate_launch_description():

    num_drones_arg = DeclareLaunchArgument(
        'num_drones',
        default_value='2'
    )

    px4_instances = OpaqueFunction(function=generate_px4_processes)

    # pequeño delay para evitar carrera entre instancias
    delayed_px4 = TimerAction(
        period=1.0,
        actions=[px4_instances]
    )

    return LaunchDescription([
        num_drones_arg,
        delayed_px4,
    ])
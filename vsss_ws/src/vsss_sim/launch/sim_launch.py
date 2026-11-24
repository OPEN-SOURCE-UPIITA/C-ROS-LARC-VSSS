"""Launch de la simulación VSSS con roles de equipo y escenarios tácticos.

Uso básico:
    ros2 launch vsss_sim sim_launch.py mi_equipo:=azul modo:=inicio

El equipo aliado siempre publica en /robotX y el rival en /rivalX,
independientemente del color ArUco que se use para cada lado.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_robots(context, *args, **kwargs):
    modo = LaunchConfiguration('modo').perform(context)
    mi_equipo = LaunchConfiguration('mi_equipo').perform(context).lower()

    ids_azul = [256, 272, 273]
    ids_amarillo = [955, 771, 939]

    if mi_equipo == 'amarillo':
        color_aliado, color_rival = 'amarillo', 'azul'
        ids_aliados, ids_rivales = ids_amarillo, ids_azul
    else:
        color_aliado, color_rival = 'azul', 'amarillo'
        ids_aliados, ids_rivales = ids_azul, ids_amarillo

    escenarios = {
        'normal': {
            'pelota': [0.0, 0.0, 0.0],
            'aliados': [[-0.5, 0.0, 0.0], [-0.5, 0.2, 0.0], [-0.5, -0.2, 0.0]],
            'rivales': [[0.5, 0.0, 3.1416], [0.5, 0.2, 3.1416], [0.5, -0.2, 3.1416]],
        },
        'inicio': {
            'pelota': [0.0, 0.0, 0.0],
            'aliados': [[-0.12, 0.0, 0.0], [-0.4, 0.3, 0.0], [-0.7, 0.0, 0.0]],
            'rivales': [[0.15, 0.0, 3.1416], [0.4, -0.3, 3.1416], [0.7, 0.0, 3.1416]],
        },
        'penalty': {
            'pelota': [0.375, 0.0, 0.0],
            'aliados': [[0.2, 0.0, 0.0], [-0.4, 0.4, 0.0], [-0.7, 0.0, 0.0]],
            'rivales': [[0.7, 0.0, 3.1416], [0.5, 0.4, 3.1416], [0.5, -0.4, 3.1416]],
        },
        'duelo': {
            'pelota': [0.15, 0.0, 0.0],
            'aliados': [[0.0, 0.0, 0.0]],
            'rivales': [[0.7, 0.0, 3.1416]],
        },
    }

    if modo not in escenarios:
        modo = 'normal'

    poses = escenarios[modo]
    pkg_vsss_sim = get_package_share_directory('vsss_sim')
    robot_sdf_path = os.path.join(pkg_vsss_sim, 'models', 'robot', 'model.sdf')
    pelota_sdf_path = os.path.join(pkg_vsss_sim, 'models', 'pelota', 'model.sdf')
    texture_dir = os.path.join(pkg_vsss_sim, 'models', 'robot', 'materials', 'textures')

    with open(robot_sdf_path, 'r', encoding='utf-8') as f:
        base_sdf = f.read()

    nodes = []
    bridge_args = ['/vsss/image@sensor_msgs/msg/Image@gz.msgs.Image']

    nodes.append(Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'pelota',
            '-file', pelota_sdf_path,
            '-x', str(poses['pelota'][0]),
            '-y', str(poses['pelota'][1]),
            '-z', '0.025',
        ],
        output='screen',
    ))

    for color, ids, side_poses, topic_prefix in (
        (color_aliado, ids_aliados, poses['aliados'], 'robot'),
        (color_rival, ids_rivales, poses['rivales'], 'rival'),
    ):
        for i, (x, y, yaw) in enumerate(side_poses):
            id_rob = ids[i]
            robot_name = f'robot_{color}_{id_rob}'
            sdf_str = base_sdf.replace('TEXTURA_ARUCO_AQUI', f'aruco_{color}_{id_rob}.png')
            sdf_str = sdf_str.replace('TOPICO_VELOCIDAD_AQUI', f'{topic_prefix}{i + 1}/cmd_vel')
            sdf_str = sdf_str.replace('materials/textures/', f'{texture_dir}/')

            tmp_path = f'/tmp/{robot_name}.sdf'
            with open(tmp_path, 'w', encoding='utf-8') as f:
                f.write(sdf_str)

            nodes.append(Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-name', robot_name,
                    '-file', tmp_path,
                    '-x', str(x),
                    '-y', str(y),
                    '-z', '0.05',
                    '-Y', str(yaw),
                ],
                output='screen',
            ))
            bridge_args.append(
                f'/{topic_prefix}{i + 1}/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
            )

    nodes.append(Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=bridge_args,
        output='screen',
    ))
    return nodes


def generate_launch_description():
    pkg_vsss_sim = get_package_share_directory('vsss_sim')
    world_file = os.path.join(pkg_vsss_sim, 'worlds', 'cancha_vsss.world')
    models_dir = os.path.join(pkg_vsss_sim, 'models')
    gz_sim = ExecuteProcess(cmd=['gz', 'sim', '-r', world_file], output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'modo',
            default_value='normal',
            description='Escenarios: normal, inicio, penalty, duelo',
        ),
        DeclareLaunchArgument(
            'mi_equipo',
            default_value='azul',
            description='Tu color en este partido (azul o amarillo)',
        ),
        AppendEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=models_dir),
        gz_sim,
        OpaqueFunction(function=generate_robots),
    ])

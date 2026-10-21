import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_vsss_sim = get_package_share_directory('vsss_sim')

    workspace_share_dir = os.path.dirname(pkg_vsss_sim)
    world_file = os.path.join(pkg_vsss_sim, 'worlds', 'cancha_vsss.world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/vsss/image@sensor_msgs/msg/Image@gz.msgs.Image',
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    return LaunchDescription([
        AppendEnvironmentVariable(
            name='GZ_SIM_RESOURCE_PATH',
            value=workspace_share_dir
        ),
        gz_sim,
        bridge
    ])
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    calibrar_arg = DeclareLaunchArgument(
        'calibrar',
        default_value='false',
        description='true para abrir ventana de calibracion, false para usar YAML guardado'
    )

    return LaunchDescription([
        calibrar_arg,

        # 1. Nodo de la Cámara
        Node(
            package='segmentation',
            executable='camera_node',
            name='camera_driver',
            output='screen',
            parameters=[{'fps': 30}]
        ),

        # 2. Nodo de Calibración
        Node(
            package='segmentation',
            executable='calibrator_node',
            name='calibrator',
            output='screen',
            parameters=[{
                'calibrar': ParameterValue(LaunchConfiguration('calibrar'), value_type=bool)
            }]
        ),

        # 3. Nodo Detector
        Node(
            package='segmentation',
            executable='detector_node',
            name='detector_vision',
            output='screen',
            parameters=[
                {'focal_length': 634.66},
                {'alpha': 0.2}
            ]
        )
    ])
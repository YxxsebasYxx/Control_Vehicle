#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # Variable para sim time
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Ruta del paquete y el archivo .xacro
    pkg_path = os.path.join(get_package_share_directory('urdf_basics'))
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')

    # Comando para convertir el archivo .xacro a URDF
    robot_description = Command(['xacro ', xacro_file])

    # Nodo para robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    # Lanzar la descripción
    return LaunchDescription([
        # Declaración de argumento para usar sim time
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use sim time if true'
        ),
        # Nodo del publicador de estado del robot
        node_robot_state_publisher
    ])

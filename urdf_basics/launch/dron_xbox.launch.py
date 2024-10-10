#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='gazebo_joy',
            parameters=[{
            'dev': '/dev/input/js0',  # Cambia jsX por el dispositivo correcto
            'deadzone': 0.12
            }],
            respawn=True
            ),

        # Nodo para teleoperar con joystick
        Node(
            package='urdf_basics',
            executable='teleop_gazebo.py',
            name='teleop',
            parameters=[{
                'axis_linear': 1,
                'axis_angular': 0,
                'scale_linear': 2.0,
                'scale_angular': 2.0
            }]
        )
        
    ])
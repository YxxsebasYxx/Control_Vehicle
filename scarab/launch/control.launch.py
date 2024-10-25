#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Joystick input node
        Node(
            package='joy',
            executable='joy_node',
            name='gazebo_joy',
            parameters=[{
                'dev': '/dev/input/js0',  # Adjust for your joystick device
                'deadzone': 0.12
            }],
            respawn=True
        ),

        # Drone teleop node
        Node(
            package='scarab',
            executable='subscriber',
            name='drone_teleop',
        ),

        #Nodo de subprocess2
        Node(
            package='scarab',
            executable='subprocess2',
            name='Process2_node',
        )
    ])

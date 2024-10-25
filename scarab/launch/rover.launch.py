#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    package_dir = get_package_share_directory('scarab')
    return LaunchDescription([
        Node(
            package='scarab',
            namespace='scarab',
            executable='rover',
            name='rover_ardupilot',
            prefix='gnome-terminal --'
        ),
        Node(
            package='scarab',
            namespace='scarab',
            executable='subprocess3',
            name='subprocess3',
            prefix='gnome-terminal --',
        ),
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    px4_path = os.getenv('PX4_PATH', default='/home/sebastian/PX4-Autopilot')
    mavros_launch_path = os.path.join(px4_path, 'launch', 'posix_sitl.launch')

    return LaunchDescription([
        # Declare Arguments
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0'),
        DeclareLaunchArgument('R', default_value='0'),
        DeclareLaunchArgument('P', default_value='0'),
        DeclareLaunchArgument('Y', default_value='0'),
        DeclareLaunchArgument('est', default_value='ekf2'),
        DeclareLaunchArgument('vehicle', default_value='iris'),
        DeclareLaunchArgument('world', default_value='empty.world'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('fcu_url', default_value='udp://:14540@localhost:14557'),

        # Include PX4 SITL
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mavros_launch_path),
            launch_arguments={
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'R': LaunchConfiguration('R'),
                'P': LaunchConfiguration('P'),
                'Y': LaunchConfiguration('Y'),
                'vehicle': LaunchConfiguration('vehicle'),
                'gui': LaunchConfiguration('gui'),
            }.items(),
        ),

        # MAVROS node
        Node(
            package='mavros',
            executable='mavros_node',
            name='mavros',
            output='screen',
            parameters=[{
                'fcu_url': LaunchConfiguration('fcu_url'),
            }]
        ),
    ])

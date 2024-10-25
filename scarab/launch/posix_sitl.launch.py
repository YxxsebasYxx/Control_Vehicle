from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Directories
    px4_path = os.path.join(get_package_share_directory('px4'))
    gazebo_ros_path = os.path.join(get_package_share_directory('gazebo_ros'))

    # Launch arguments
    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0'),
        DeclareLaunchArgument('R', default_value='0'),
        DeclareLaunchArgument('P', default_value='0'),
        DeclareLaunchArgument('Y', default_value='0'),
        DeclareLaunchArgument('vehicle', default_value='iris'),
        DeclareLaunchArgument('world', default_value=os.path.join(px4_path, 'worlds', 'empty.world')),

        # PX4 SITL Node
        Node(
            package='px4',
            executable='px4',
            name='sitl',
            output='screen',
            arguments=[os.path.join(px4_path, 'build', 'px4_sitl_default', 'etc'), '-s', 'etc/init.d-posix/rcS']
        ),

        # Gazebo simulation
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')]),
            launch_arguments={'world': LaunchConfiguration('world')}.items()
        ),

        # Gazebo model spawn
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_vehicle',
            output='screen',
            arguments=[
                '-entity', LaunchConfiguration('vehicle'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y')
            ]
        )
    ])

from setuptools import setup
import os
from glob import glob

package_name = 'px4_ros_com'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='sebastian@todo.todo',
    description='PX4 ROS 2 package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_teleop_joy = px4_ros_com.turtle_teleop_joy:main',
            'joy_node = px4_ros_com.joy_node:main',
        ],
    },
)


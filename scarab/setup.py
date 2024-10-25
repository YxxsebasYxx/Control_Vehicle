from setuptools import find_packages, setup

package_name = 'scarab'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/movement.launch.py']),
        ('share/' + package_name + '/launch', ['launch/control.launch.py']),
        ('share/' + package_name + '/launch', ['launch/mavros_posix_sitl.launch.py']),
        ('share/' + package_name + '/launch', ['launch/posix_sitl.launch.py']),
        ('share/' + package_name + '/launch', ['launch/rover.launch.py']),
        ('share/' + package_name + '/launch', ['launch/turtle_xbox.launch.py']),
        ('share/' + package_name + '/launch', ['launch/teleop_dron.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sebastian',
    maintainer_email='sebastian@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joystick_handler = scarab.joystick_handler:main",
            "rover = scarab.rover:main",
            "subprocess = scarab.subprocess:main",
            "movement = scarab.movement:main",
            "subscriber = scarab.subscriber:main",
            "subprocess2 = scarab.subprocess2:main",
            "subprocess3 = scarab.subprocess3:main",
            "turtle_teleop_joy = scarab.turtle_teleop_joy:main",
            "joy_node = scarab.joy_node:main",
        ],
    },
)

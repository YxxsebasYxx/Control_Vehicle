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
            "listen = scarab.listen:main",
            "arm = scarab.arm:main",
            "subprocess = scarab.subprocess:main",
            "movement = scarab.movement:main",
        ],
    },
)

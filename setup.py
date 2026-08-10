import os
from setuptools import find_packages, setup
from glob import glob


package_name = 'rx3_robot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
        glob(os.path.join('launch', '*.launch.py'))),
        (os.path.join('share', package_name, 'config'),
        glob(os.path.join('config', '*.yaml'))),   # <-- agregar esta línea
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arrgusers',
    maintainer_email='tareas2inge@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
    'console_scripts': [
        'rosmaster_bridge = rx3_robot_bridge.rosmaster_bridge_node:main',
        'calibrate_arm = rx3_robot_bridge.calibrate_arm:main', 
        'hw_pid_battery = rx3_robot_bridge.hw_pid_battery:main',
        ],
    },
)

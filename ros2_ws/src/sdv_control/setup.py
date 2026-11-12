import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'sdv_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='togo-matsu',
    maintainer_email='you@example.com',
    description='ROS 2 x QNX SDV Telemetry, TF2 Odometry, and ADAS Safety Override Package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'telemetry_node = sdv_control.telemetry_node:main',
            'adas_safety_node = sdv_control.adas_safety_node:main',
            'emqx_bridge_node = sdv_control.emqx_bridge_node:main',
        ],
    },
)

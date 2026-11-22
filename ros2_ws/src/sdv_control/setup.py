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
            'tello_teleop_node = sdv_control.tello_teleop_node:main',
            'tello_ros2_node = sdv_control.tello_ros2_node:main',
            'telemetry_node = sdv_control.telemetry_node:main',
            'adas_safety_node = sdv_control.adas_safety_node:main',
            'emqx_bridge_node = sdv_control.emqx_bridge_node:main',
            'stereo_depth_fusion_node = sdv_control.stereo_depth_fusion_node:main',
            'vio_imu_node = sdv_control.vio_imu_node:main',
            'stereo_avoidance_node = sdv_control.stereo_avoidance_node:main',
            'px4_stereo_avoidance_node = sdv_control.px4_stereo_avoidance_node:main',
            'drone_offboard_commander = sdv_control.drone_offboard_commander:main',
            'drone_mavlink_ros2_node = sdv_control.drone_mavlink_ros2_node:main',
            'sim_qnx_bridge_node = sdv_control.sim_qnx_bridge_node:main',
            'pillar_nav_node = sdv_control.pillar_nav_node:main',
        ],
    },
)

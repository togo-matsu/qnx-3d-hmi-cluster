from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sdv_control',
            executable='telemetry_node',
            name='sdv_telemetry_node',
            output='screen'
        ),
        Node(
            package='sdv_control',
            executable='adas_safety_node',
            name='adas_safety_node',
            output='screen'
        ),
        Node(
            package='sdv_control',
            executable='emqx_bridge_node',
            name='ros2_emqx_bridge_node',
            output='screen'
        ),
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/vehicle/odom',
        description='Odometry topic for emqx_bridge_node'
    )

    return LaunchDescription([
        odom_topic_arg,
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
            parameters=[{'odom_topic': LaunchConfiguration('odom_topic')}],
            output='screen'
        ),
    ])

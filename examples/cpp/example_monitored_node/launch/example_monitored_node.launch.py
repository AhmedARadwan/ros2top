"""Launch file for the example monitored C++ node."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='example_monitored_node',
            executable='example_monitored_node',
            name='example_monitored_node_cpp',
            output='screen',
            parameters=[],
            remappings=[
                ('example_topic_cpp', 'example_topic_cpp'),
                ('example_input_cpp', 'example_input_cpp'),
            ]
        )
    ])

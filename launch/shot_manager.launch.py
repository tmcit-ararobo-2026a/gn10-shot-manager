"""Launch file for gn10_shot_manager recorder."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gn10_shot_manager',
            executable='trial_recorder',
            name='trial_recorder_node',
            output='screen',
            parameters=[{
                'record_duration_sec': 3.5,
                'pre_trigger_buffer_sec': 0.5,
            }]
        )
    ])


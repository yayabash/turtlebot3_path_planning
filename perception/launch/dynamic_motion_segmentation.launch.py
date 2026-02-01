from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='dynamic_motion_segmentation.py',
            name='dynamic_motion_segmentation',
            output='screen',
        )
    ])

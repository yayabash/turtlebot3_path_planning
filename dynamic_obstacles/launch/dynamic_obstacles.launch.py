from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        TimerAction(
            period=3.0,  # wait for Gazebo to finish loading
            actions=[
                Node(
                    package='dynamic_obstacles',
                    executable='obstacle_manager',
                    name='obstacle_manager',
                    output='screen'
                )
            ]
        )
    ])


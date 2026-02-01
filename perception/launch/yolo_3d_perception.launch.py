from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the YOLO 3D Detector Node
        Node(
            package='perception',
            executable='yolo_3d_detector.py',
            name='yolo_3d_detector',
            output='screen',
            parameters=[{'model_path': 'yolov8n.pt'}]
        ),
        
        # 2. Start RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            # We will save a config later, for now let it open empty
        ),
    ])
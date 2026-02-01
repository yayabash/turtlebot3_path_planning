from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzrgbNode',
                name='point_cloud_xyzrgb_node',
                remappings=[
                    ('rgb/image_rect_color', '/camera/image_raw'),
                    ('rgb/camera_info', '/camera/camera_info'),
                    ('depth_registered/image_rect', '/camera/depth/image_raw'),
                    ('points', '/camera/depth_registered/points'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_nav2 = get_package_share_directory('turtlebot3_navigation2')
    nav2_launch = os.path.join(pkg_nav2, 'launch', 'navigation2.launch.py')

    default_params = os.path.join(pkg_nav2, 'param', 'waffle_astar.yaml')

    declare_map = DeclareLaunchArgument('map', default_value=TextSubstitution(text=''))
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='True')
    declare_params = DeclareLaunchArgument('params_file', default_value=default_params)

    include_nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file')
        }.items()
    )

    return LaunchDescription([
        declare_map,
        declare_use_sim_time,
        declare_params,
        include_nav2,
    ])

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    this_pkg_share_dir = get_package_share_directory("wheelchair_control_support")
    list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [this_pkg_share_dir,'/launch', '/whill_modelc.launch.py']
                )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [this_pkg_share_dir, '/launch', '/start_2_lidar.launch.py']
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [this_pkg_share_dir, '/launch', '/merge_lidar.launch.py']
            )
        ),
        Node(
            package='wheelchair_control_support',
            executable='share_control_node',
            name='share_control_node',
            output='screen',
            parameters=[
                os.path.join(this_pkg_share_dir, "config", "share_control_config.yaml")
            ]
            ),
    ]
    
    return LaunchDescription(list)

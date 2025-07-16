from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the installed map.yaml file inside your package
    pkg_share = get_package_share_directory('wheelchair_control_support')
    map_file = os.path.join(pkg_share, 'map', 'living_lab.yaml')
    amcl_params_file = os.path.join(pkg_share, 'config', 'living_lab_localization_amcl.yaml')

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}]
        ),
        
           # AMCL localization node
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[amcl_params_file]
        )
    ])


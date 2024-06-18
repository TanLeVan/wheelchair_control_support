import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    whill_pkg_dir = get_package_share_directory("ros2_whill")
    this_pkg_share_dir = get_package_share_directory("wheelchair_control_support")

    urdf_file = os.path.join(this_pkg_share_dir, 'model', 'modelc_with_lidar.urdf')
    with open(urdf_file, 'r') as infp:
            robot_desc = infp.read()
    params = {'robot_description': robot_desc}

    list = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [whill_pkg_dir, '/launch', '/modelc.launch.py']
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[params],
            remappings=[
                ('/joint_states', '/whill/states/joint_state')
            ]
        ),

    ]
    
    return LaunchDescription(list)

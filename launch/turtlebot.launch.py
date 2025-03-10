import yaml
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    this_pkg_share_dir = get_package_share_directory("wheelchair_control_support")
    share_control_config_path =  os.path.join(this_pkg_share_dir, "config", "turtlebot_share_control_config.yaml") #path of the config file for share control
    with open(share_control_config_path, 'r') as f:
        config_param = yaml.safe_load(f)


    

    # Define paths to packages and configuration
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
 
        # Include TurtleBot3 Gazebo launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                turtlebot3_gazebo, '/launch/turtlebot3_house.launch.py'
            ]),
        ),

        # Node for joystick control (simulated or real)
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Node for ShareControl
        Node(
            package='wheelchair_control_support',
            executable='share_control_node',
            name='share_control_node',
            output='screen',
            parameters=[
                share_control_config_path
            ],
            remappings=[
                ('/scan', '/scan'),  # Assuming turtlebot3 publishes scan under this topic
                ('/whill/odom', '/odom'),  # TurtleBot3 odometry
                ('/whill/states/joy', '/joy'),  # Joystick topic
                ('/whill/controller/cmd_vel', '/cmd_vel')  # Output velocity command topic
            ],
        ),
    ])

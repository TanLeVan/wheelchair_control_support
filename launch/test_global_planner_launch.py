import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

RAD_TO_SPEED_SETTING = 10/0.561  #ratio of the setting tm in whill to the actual angular vel
METER_TO_SPEED_SETTING = 36 #ratio of setting of linear speed in whill to actual linear vel

def generate_launch_description():
    this_pkg_share_dir = get_package_share_directory("wheelchair_control_support")
    test_gp =  os.path.join(this_pkg_share_dir, "config", "test_global_planner.yaml") #path of the config file for share control
    with open(test_gp, 'r') as f:
        config_param = yaml.safe_load(f)

    # whill_max_linear_speed = min(max(int(config_param["shared_controller_node"]["ros__parameters"]["max_linear_vel"] * METER_TO_SPEED_SETTING), 8), 60)  # max forward speed
    # whill_min_linear_speed = min(max(int(abs(int(config_param["shared_controller_node"]["ros__parameters"]["min_linear_vel"] * METER_TO_SPEED_SETTING))), 8), 60)  # max backward speed
    # whill_max_yaw_rate = min(max(int(config_param["shared_controller_node"]["ros__parameters"]["max_yaw_rate"] * RAD_TO_SPEED_SETTING), 8), 60)  # max angular speed
    # whill_max_acceleration = min(max(int(config_param["shared_controller_node"]["ros__parameters"]["max_acceleration"] * METER_TO_SPEED_SETTING), 10), 90)  # Max forward acceleration (backward deceleration)
    # whill_max_deceleration = min(max(int(config_param["shared_controller_node"]["ros__parameters"]["max_decceleration"] * METER_TO_SPEED_SETTING), 10), 160)  # Max forward deceleration (backward acceleration)
    # whill_max_yaw_acceleration = min(max(int(config_param["shared_controller_node"]["ros__parameters"]["max_yaw_acceleration"] * RAD_TO_SPEED_SETTING), 10), 90)  # Max yaw rate acceleration
    
    # service_command = [
    #     'ros2', 'service', 'call', '/whill/set_speed_profile_srv', 'ros2_whill_interfaces/SetSpeedProfile',
    #     "{s1: 4, fm1: " + str(whill_max_linear_speed) + ", fa1: " + str(whill_max_acceleration) + ", fd1: " + str(whill_max_deceleration) +
    #     ", rm1: " + str(whill_min_linear_speed) + ", ra1: " + str(whill_max_acceleration) + ", rd1: " + str(whill_max_deceleration) +
    #     ", tm1: " + str(whill_max_yaw_rate) + ", ta1: " + str(whill_max_yaw_acceleration) + ", td1: " + str(whill_max_yaw_acceleration) + "}"
    # ]
    # print(service_command)

    list = [

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [this_pkg_share_dir,'/launch', '/whill_modelc.launch.py']
        #         )
        # ),
        # # ExecuteProcess(
        # #     cmd=service_command
        # # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [this_pkg_share_dir, '/launch', '/start_2_lidar.launch.py']
        #     )
        # ),
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         [this_pkg_share_dir, '/launch', '/merge_lidar.launch.py']
        #     )
        # ),
        Node(
            package='nav2_planner', executable='planner_server', 
            name='planner_server',
            output='screen',
            parameters= [
                test_gp
            ],  # Load parameters from YAML
        ),
         # Static transform publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            output='screen',
            arguments=[
                '0.0',  # x translation (meters)
                '0.0',  # y translation (meters)
                '0.0',  # z translation (meters)
                '0.0',  # roll (radians)
                '0.0',  # pitch (radians)
                '0.0',  # yaw (radians)
                'base_link',  # parent frame
                'map'  # child frame
            ]
        ),
    ]
    
    return LaunchDescription(list)
